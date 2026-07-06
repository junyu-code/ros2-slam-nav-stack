#!/usr/bin/env python3

import argparse
import base64
import http.client
import json
import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path


REPO_OWNER = 'agilexrobotics'
REPO_NAME = 'agilex_open_class'
DEFAULT_REF = 'master'
DEFAULT_PACKAGES = [
    'piper_description',
    'piper_moveit_config_v4',
    'piper_moveit_config_v5',
]
USER_AGENT = 'slam-nav-ws-piper-downloader'


class RateLimitExceeded(RuntimeError):
    pass


def parse_args():
    parser = argparse.ArgumentParser(description='只下载 AgileX open class 的 Piper 官方示例包。')
    parser.add_argument('--output-dir', default='external/agilex/agilex_open_class')
    parser.add_argument('--ref', default=DEFAULT_REF)
    parser.add_argument('--force', action='store_true', help='强制重新下载已存在文件。')
    parser.add_argument(
        '--package',
        action='append',
        dest='packages',
        help='只下载指定 piper 子包；可重复传入。默认下载 description 和 v4/v5 MoveIt2 配置。',
    )
    parser.add_argument('--retries', type=int, default=4)
    parser.add_argument('--retry-sleep', type=float, default=2.0)
    parser.add_argument('--wait-rate-limit', action='store_true', help='GitHub API 额度用尽时自动等到 reset 后继续。')
    parser.add_argument(
        '--file-backend',
        choices=['raw', 'blob', 'blob-json'],
        default='blob',
        help='文件下载后端；blob 使用 GitHub API 原始二进制，blob-json 使用 base64 JSON，raw 使用 raw.githubusercontent.com。',
    )
    parser.add_argument('--chunk-size', type=int, default=393216, help='blob 后端下载大文件时的 Range 分块大小。')
    return parser.parse_args()


def github_headers():
    headers = {
        'Accept': 'application/vnd.github+json',
        'User-Agent': USER_AGENT,
        'X-GitHub-Api-Version': '2022-11-28',
    }
    token = os.environ.get('GITHUB_TOKEN')
    if token:
        headers['Authorization'] = f'Bearer {token}'
    return headers


def open_with_retry(url, *, headers=None, retries=4, retry_sleep=2.0):
    headers = headers or {}
    last_error = None
    for attempt in range(1, retries + 1):
        request = urllib.request.Request(url, headers=headers)
        try:
            return urllib.request.urlopen(request, timeout=60)
        except urllib.error.HTTPError as exc:
            last_error = exc
            if exc.code not in {429, 500, 502, 503, 504} or attempt == retries:
                raise
            wait_s = retry_sleep * attempt
            print(f'[Piper] GitHub 返回 HTTP {exc.code}，{wait_s:.1f}s 后重试: {url}', file=sys.stderr)
            time.sleep(wait_s)
        except urllib.error.URLError as exc:
            last_error = exc
            if attempt == retries:
                raise
            wait_s = retry_sleep * attempt
            print(f'[Piper] 网络暂时不可用，{wait_s:.1f}s 后重试: {exc}', file=sys.stderr)
            time.sleep(wait_s)
    raise RuntimeError(f'下载失败: {last_error}')


def rate_limit_wait_seconds(exc):
    remaining = exc.headers.get('X-RateLimit-Remaining')
    reset_at = exc.headers.get('X-RateLimit-Reset')
    if exc.code != 403 or remaining != '0' or not reset_at:
        return None
    return max(0, int(reset_at) - int(time.time())) + 2


def maybe_wait_rate_limit(exc, *, wait_rate_limit):
    wait_s = rate_limit_wait_seconds(exc)
    if wait_s is None:
        return False
    if wait_rate_limit:
        print(f'[Piper] GitHub API 额度用尽，等待 {wait_s}s 后继续...', file=sys.stderr, flush=True)
        time.sleep(wait_s)
        return True
    raise RateLimitExceeded(
        f'GitHub API 额度用尽，约 {wait_s}s 后重试；也可以设置 GITHUB_TOKEN 或传入 --wait-rate-limit。'
    )


def load_json_with_retry(url, *, headers=None, retries=4, retry_sleep=2.0, wait_rate_limit=False):
    headers = headers or {}
    last_error = None
    for attempt in range(1, retries + 1):
        request = urllib.request.Request(url, headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                return json.loads(response.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            last_error = exc
            if maybe_wait_rate_limit(exc, wait_rate_limit=wait_rate_limit):
                continue
            if exc.code not in {429, 500, 502, 503, 504} or attempt == retries:
                raise
            wait_s = retry_sleep * attempt
            print(f'[Piper] GitHub 返回 HTTP {exc.code}，{wait_s:.1f}s 后重试: {url}', file=sys.stderr, flush=True)
            time.sleep(wait_s)
        except (urllib.error.URLError, http.client.IncompleteRead, json.JSONDecodeError) as exc:
            last_error = exc
            if attempt == retries:
                raise
            wait_s = retry_sleep * attempt
            print(f'[Piper] 响应读取失败，{wait_s:.1f}s 后重试: {exc}', file=sys.stderr, flush=True)
            time.sleep(wait_s)
    raise RuntimeError(f'读取 JSON 失败: {last_error}')


def read_binary_with_retry(url, *, headers=None, retries=4, retry_sleep=2.0, expected_size=0, wait_rate_limit=False):
    headers = headers or {}
    last_error = None
    for attempt in range(1, retries + 1):
        request = urllib.request.Request(url, headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=90) as response:
                content = response.read()
            if expected_size > 0 and len(content) != expected_size:
                raise RuntimeError(f'文件大小不匹配，期望 {expected_size}，实际 {len(content)}')
            return content
        except urllib.error.HTTPError as exc:
            last_error = exc
            if maybe_wait_rate_limit(exc, wait_rate_limit=wait_rate_limit):
                continue
            if exc.code not in {429, 500, 502, 503, 504} or attempt == retries:
                raise
            wait_s = retry_sleep * attempt
            print(f'[Piper] GitHub 返回 HTTP {exc.code}，{wait_s:.1f}s 后重试: {url}', file=sys.stderr, flush=True)
            time.sleep(wait_s)
        except (urllib.error.URLError, http.client.IncompleteRead, RuntimeError) as exc:
            last_error = exc
            if attempt == retries:
                raise
            wait_s = retry_sleep * attempt
            print(f'[Piper] 文件读取失败，{wait_s:.1f}s 后重试: {exc}', file=sys.stderr, flush=True)
            time.sleep(wait_s)
    raise RuntimeError(f'读取文件失败: {last_error}')


def fetch_tree(ref, retries, retry_sleep, wait_rate_limit):
    url = f'https://api.github.com/repos/{REPO_OWNER}/{REPO_NAME}/git/trees/{ref}?recursive=1'
    data = load_json_with_retry(
        url,
        headers=github_headers(),
        retries=retries,
        retry_sleep=retry_sleep,
        wait_rate_limit=wait_rate_limit,
    )
    if data.get('truncated'):
        raise RuntimeError('GitHub tree API 返回被截断，不能可靠下载 Piper 子目录。')
    return data['tree']


def select_files(tree, packages):
    prefixes = tuple(f'piper/{package}/' for package in packages)
    files = [
        item
        for item in tree
        if item.get('type') == 'blob' and item.get('path', '').startswith(prefixes)
    ]
    return sorted(files, key=file_sort_key)


def file_sort_key(item):
    path = item['path']
    parts = path.split('/')
    filename = parts[-1]
    rel_parts = parts[2:]
    top_dir = rel_parts[0] if len(rel_parts) > 1 else ''

    if filename in {'package.xml', 'CMakeLists.txt'}:
        priority = 0
    elif top_dir in {'urdf', 'config', 'launch'}:
        priority = 1
    elif top_dir in {'rviz', 'worlds'}:
        priority = 2
    elif top_dir == 'meshes':
        priority = 9
    else:
        priority = 5
    return priority, path


def raw_url(ref, repo_path):
    quoted_path = urllib.parse.quote(repo_path, safe='/')
    return f'https://raw.githubusercontent.com/{REPO_OWNER}/{REPO_NAME}/{ref}/{quoted_path}'


def blob_url(item):
    return item['url']


def read_blob_bytes(item, args):
    data = load_json_with_retry(
        blob_url(item),
        headers=github_headers(),
        retries=args.retries,
        retry_sleep=args.retry_sleep,
        wait_rate_limit=args.wait_rate_limit,
    )
    if data.get('encoding') != 'base64':
        raise RuntimeError(f'GitHub blob 编码不是 base64: {item["path"]}')
    return base64.b64decode(data['content'])


def read_blob_raw_bytes(item, args):
    headers = github_headers()
    headers['Accept'] = 'application/vnd.github.raw'
    expected_size = int(item.get('size', 0))
    if expected_size > args.chunk_size > 0:
        chunks = []
        for start in range(0, expected_size, args.chunk_size):
            end = min(start + args.chunk_size, expected_size) - 1
            chunk_headers = dict(headers)
            chunk_headers['Range'] = f'bytes={start}-{end}'
            chunks.append(
                read_binary_with_retry(
                    blob_url(item),
                    headers=chunk_headers,
                    retries=args.retries,
                    retry_sleep=args.retry_sleep,
                    expected_size=end - start + 1,
                    wait_rate_limit=args.wait_rate_limit,
                )
            )
        return b''.join(chunks)
    return read_binary_with_retry(
        blob_url(item),
        headers=headers,
        retries=args.retries,
        retry_sleep=args.retry_sleep,
        expected_size=expected_size,
        wait_rate_limit=args.wait_rate_limit,
    )


def read_raw_bytes(item, args):
    url = raw_url(args.ref, item['path'])
    headers = {'User-Agent': USER_AGENT}
    return read_binary_with_retry(
        url,
        headers=headers,
        retries=args.retries,
        retry_sleep=args.retry_sleep,
        expected_size=int(item.get('size', 0)),
        wait_rate_limit=args.wait_rate_limit,
    )


def download_file(item, destination, args):
    expected_size = int(item.get('size', 0))
    if destination.exists() and not args.force:
        if expected_size <= 0 or destination.stat().st_size == expected_size:
            return 'skip'

    destination.parent.mkdir(parents=True, exist_ok=True)
    temp_path = destination.with_suffix(destination.suffix + '.download')

    if args.file_backend == 'blob':
        content = read_blob_raw_bytes(item, args)
    elif args.file_backend == 'blob-json':
        content = read_blob_bytes(item, args)
    else:
        content = read_raw_bytes(item, args)

    temp_path.write_bytes(content)

    actual_size = temp_path.stat().st_size
    if expected_size > 0 and actual_size != expected_size:
        temp_path.unlink(missing_ok=True)
        raise RuntimeError(
            f'文件大小不匹配: {item["path"]}, 期望 {expected_size}, 实际 {actual_size}'
        )
    temp_path.replace(destination)
    return 'download'


def write_manifest(output_dir, ref, packages, files, *, complete, failed_path=''):
    existing_paths = []
    missing_paths = []
    for item in files:
        path = output_dir / item['path']
        if path.exists() and (not item.get('size') or path.stat().st_size == int(item.get('size', 0))):
            existing_paths.append(item['path'])
        else:
            missing_paths.append(item['path'])

    manifest = {
        'source': f'https://github.com/{REPO_OWNER}/{REPO_NAME}/tree/{ref}/piper',
        'ref': ref,
        'packages': packages,
        'complete': complete,
        'failed_path': failed_path,
        'file_count': len(files),
        'total_size': sum(int(item.get('size', 0)) for item in files),
        'existing_count': len(existing_paths),
        'missing_count': len(missing_paths),
        'existing_paths': existing_paths,
        'missing_paths': missing_paths,
        'paths': [item['path'] for item in files],
    }
    manifest_path = output_dir / '.piper_open_class_manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    packages = args.packages or DEFAULT_PACKAGES
    output_dir = Path(args.output_dir)

    try:
        print(f'[Piper] 读取 AgileX open class tree: {REPO_OWNER}/{REPO_NAME}@{args.ref}', flush=True)
        tree = fetch_tree(args.ref, args.retries, args.retry_sleep, args.wait_rate_limit)
        files = select_files(tree, packages)
        if not files:
            print(f'[Piper] 未找到 Piper 包: {", ".join(packages)}', file=sys.stderr)
            return 2

        total_size = sum(int(item.get('size', 0)) for item in files)
        print(f'[Piper] 准备下载 {len(files)} 个文件，约 {total_size / 1024 / 1024:.1f} MiB。', flush=True)

        downloaded = 0
        skipped = 0
        failed_path = ''
        try:
            for index, item in enumerate(files, 1):
                destination = output_dir / item['path']
                action = download_file(item, destination, args)
                if action == 'skip':
                    skipped += 1
                else:
                    downloaded += 1
                print(f'[Piper] {index:02d}/{len(files):02d} {action:8s} {item["path"]}', flush=True)
        except Exception as exc:
            failed_path = item['path']
            write_manifest(output_dir, args.ref, packages, files, complete=False, failed_path=failed_path)
            raise exc

        write_manifest(output_dir, args.ref, packages, files, complete=True)
        print(f'[Piper] 下载完成：新增/更新 {downloaded}，跳过 {skipped}。', flush=True)
        return 0
    except Exception as exc:
        print(f'[Piper] 下载失败：{exc}', file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    sys.exit(main())
