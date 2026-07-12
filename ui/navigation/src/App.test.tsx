import { act, render, screen, waitFor, within } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { afterEach, beforeEach, describe, expect, test, vi } from 'vitest'
import App from './App'

const originalWebSocket = globalThis.WebSocket
const originalFetch = globalThis.fetch
const originalMediaRecorder = globalThis.MediaRecorder
const originalCaptureStream = HTMLCanvasElement.prototype.captureStream
const originalCreateObjectUrl = URL.createObjectURL
const originalRevokeObjectUrl = URL.revokeObjectURL
const defaultRgbWsUrl = () => `ws://${window.location.host}/ws/rgb`
const defaultNavWsUrl = () => `ws://${window.location.host}/ws/nav`
const defaultNavDepthWsUrl = () => `ws://${window.location.host}/ws/nav/depth`
const defaultPiperRgbWsUrl = () => `ws://${window.location.host}/ws/piper/rgb`
const defaultPiperDepthWsUrl = () => `ws://${window.location.host}/ws/piper/depth`

function createTaskState() {
  return {
    running: true,
    current: createRun('run-mapping', 'auto-mapping', '自动建图', null),
    active: [createRun('run-mapping', 'auto-mapping', '自动建图', null)],
    history: [createRun('run-check', 'ui-gui-check', '图形自检', 0)],
    flows: [
      {
        name: '演示启动',
        items: [
          {
            id: 'operator',
            label: '原生 Operator',
            summary: '打开嵌入 RViz 的 Qt 专业控制台。',
            level: 'primary',
            command: './run.sh operator',
          },
          {
            id: 'nav',
            label: '自主导航',
            summary: '加载默认地图并启动 Nav2 导航链路。',
            level: 'primary',
            command: './run.sh nav',
          },
        ],
      },
      {
        name: '检查',
        items: [
          {
            id: 'ui-gui-check',
            label: '图形自检',
            summary: '检查图形入口。',
            level: 'check',
            command: './run.sh ui-gui-check',
          },
        ],
      },
    ],
    operator: { available: true, running: false, reason: null as string | null },
    now: 1_720_000_000,
  }
}

function createRun(runId: string, flowId: string, label: string, returnCode: number | null) {
  const finished = returnCode === null ? null : 1_720_000_000
  return {
    runId,
    flowId,
    label,
    command: [`./run.sh`, flowId],
    startedAt: 1_719_999_900,
    finishedAt: finished,
    returnCode,
    stopped: false,
    logPath: `log/ui/${runId}.log`,
  }
}

function jsonResponse(payload: unknown, status = 200) {
  return new Response(JSON.stringify(payload), {
    status,
    headers: { 'Content-Type': 'application/json' },
  })
}

let taskState = createTaskState()
let fetchMock: ReturnType<typeof vi.fn>

class MockWebSocket {
  static instances: MockWebSocket[] = []

  onopen: (() => void) | null = null
  onclose: ((event: CloseEvent) => void) | null = null
  onerror: (() => void) | null = null
  onmessage: ((event: MessageEvent) => void) | null = null
  readonly url: string
  readyState: number = WebSocket.OPEN
  sent: string[] = []

  constructor(url: string) {
    this.url = url
    MockWebSocket.instances.push(this)
  }

  close() {
    this.readyState = WebSocket.CLOSED
    this.onclose?.({ code: 1000, reason: 'client closed' } as CloseEvent)
  }

  send(message: string) {
    this.sent.push(message)
  }
}

beforeEach(() => {
  taskState = createTaskState()
  window.history.replaceState({}, '', '/')
  fetchMock = vi.fn(async (input: RequestInfo | URL, options?: RequestInit) => {
    const path = String(input)
    if (path === '/api/state') {
      return jsonResponse(taskState)
    }
    if (path.startsWith('/api/log?')) {
      return jsonResponse({ text: '[slam_nav_ws] flow output' })
    }
    if (path === '/api/run') {
      const body = JSON.parse(String(options?.body ?? '{}')) as { flowId?: string }
      const run = createRun(`run-${body.flowId}`, body.flowId ?? '', body.flowId === 'operator' ? '原生 Operator' : '自主导航', null)
      taskState = {
        ...taskState,
        running: true,
        current: run,
        active: [...taskState.active, run],
        operator: body.flowId === 'operator'
          ? { available: true, running: true, reason: null }
          : taskState.operator,
      }
      return jsonResponse({ ok: true, current: run }, 202)
    }
    if (path === '/api/stop') {
      return jsonResponse({ ok: true, stopped: 1 })
    }
    return jsonResponse({ error: 'not found' }, 404)
  })
  Object.defineProperty(globalThis, 'fetch', {
    configurable: true,
    writable: true,
    value: fetchMock,
  })
  vi.spyOn(HTMLCanvasElement.prototype, 'getContext').mockReturnValue({
    arc: vi.fn(),
    beginPath: vi.fn(),
    clearRect: vi.fn(),
    closePath: vi.fn(),
    createImageData: vi.fn((width: number, height: number) => ({
      data: new Uint8ClampedArray(width * height * 4),
      width,
      height,
    })),
    drawImage: vi.fn(),
    fill: vi.fn(),
    fillRect: vi.fn(),
    fillText: vi.fn(),
    lineTo: vi.fn(),
    moveTo: vi.fn(),
    putImageData: vi.fn(),
    restore: vi.fn(),
    rotate: vi.fn(),
    save: vi.fn(),
    stroke: vi.fn(),
    translate: vi.fn(),
  } as unknown as CanvasRenderingContext2D)
})

afterEach(() => {
  globalThis.WebSocket = originalWebSocket
  Object.defineProperty(globalThis, 'fetch', {
    configurable: true,
    writable: true,
    value: originalFetch,
  })
  window.history.replaceState({}, '', '/')
  Object.defineProperty(globalThis, 'MediaRecorder', {
    configurable: true,
    writable: true,
    value: originalMediaRecorder,
  })
  Object.defineProperty(HTMLCanvasElement.prototype, 'captureStream', {
    configurable: true,
    writable: true,
    value: originalCaptureStream,
  })
  Object.defineProperty(URL, 'createObjectURL', {
    configurable: true,
    writable: true,
    value: originalCreateObjectUrl,
  })
  Object.defineProperty(URL, 'revokeObjectURL', {
    configurable: true,
    writable: true,
    value: originalRevokeObjectUrl,
  })
  MockWebSocket.instances = []
  vi.restoreAllMocks()
  vi.useRealTimers()
})

describe('SLAM Nav navigation workstation', () => {
  test('auto-connects to camera and navigation websocket bridges when the page loads', async () => {
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket

    render(<App />)

    expect(screen.getByRole('heading', { name: 'SLAM Nav 导航作业台' })).toBeInTheDocument()
    expect(screen.getByLabelText('相机桥')).toHaveValue(defaultRgbWsUrl())
    expect(screen.getByLabelText('导航桥')).toHaveValue(defaultNavWsUrl())
    await waitFor(() => expect(MockWebSocket.instances).toHaveLength(5))
    expect(MockWebSocket.instances.map((socket) => socket.url).sort()).toEqual([
      defaultRgbWsUrl(),
      defaultNavWsUrl(),
      defaultNavDepthWsUrl(),
      defaultPiperRgbWsUrl(),
      defaultPiperDepthWsUrl(),
    ].sort())
    expect(screen.getByRole('button', { name: '暂停' })).toBeEnabled()
    expect(screen.getByRole('button', { name: '开始导航' })).toBeDisabled()
  })

  test('allows pausing and resuming the camera preview state after auto-connect', async () => {
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket
    const user = userEvent.setup()
    render(<App />)

    await user.click(screen.getByRole('button', { name: '暂停' }))

    expect(screen.getByRole('button', { name: '恢复' })).toBeEnabled()
    expect(screen.getByText('画面已暂停')).toBeInTheDocument()

    await user.click(screen.getByRole('button', { name: '恢复' }))

    expect(screen.getByRole('button', { name: '暂停' })).toBeEnabled()
  })

  test('automatically reconnects the camera bridge when its websocket closes unexpectedly', async () => {
    vi.useFakeTimers()
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket

    render(<App />)
    expect(MockWebSocket.instances).toHaveLength(5)

    const cameraSocket = MockWebSocket.instances.find((socket) => socket.url === defaultRgbWsUrl())
    act(() => {
      cameraSocket?.onclose?.({
        code: 1006,
        reason: 'network lost',
      } as CloseEvent)
      vi.advanceTimersByTime(1_500)
    })

    expect(MockWebSocket.instances).toHaveLength(6)
    expect(MockWebSocket.instances.filter((socket) => socket.url === defaultRgbWsUrl())).toHaveLength(2)
  })

  test('enables navigation only after the navigation ready topic becomes true', async () => {
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket
    const user = userEvent.setup()
    render(<App />)

    await waitFor(() => expect(MockWebSocket.instances).toHaveLength(5))
    const navSocket = MockWebSocket.instances.find((socket) => socket.url === defaultNavWsUrl())
    const mapCanvas = screen.getByLabelText('可标点导航地图')
    vi.spyOn(mapCanvas, 'getBoundingClientRect').mockReturnValue({
      x: 0,
      y: 0,
      left: 0,
      top: 0,
      right: 400,
      bottom: 400,
      width: 400,
      height: 400,
      toJSON: () => ({}),
    })

    act(() => {
      navSocket?.onopen?.()
      navSocket?.onmessage?.({
        data: JSON.stringify({
          type: 'map',
          width: 10,
          height: 10,
          resolution: 1,
          origin: { x: 0, y: 0, yaw: 0 },
          data: Array.from({ length: 100 }, () => 0),
          topic: '/map',
        }),
      } as MessageEvent)
      navSocket?.onmessage?.({
        data: JSON.stringify({ type: 'navigation_ready', ready: false, topic: '/navigation_ready' }),
      } as MessageEvent)
    })

    await user.click(mapCanvas)
    expect(screen.getByText('未就绪')).toBeInTheDocument()
    expect(screen.getByRole('button', { name: '开始导航' })).toBeDisabled()

    act(() => {
      navSocket?.onmessage?.({
        data: JSON.stringify({ type: 'navigation_ready', ready: true, topic: '/navigation_ready' }),
      } as MessageEvent)
    })

    expect(screen.getByText('可以开始')).toBeInTheDocument()
    expect(screen.getByText('导航已就绪')).toBeInTheDocument()
    const startButton = screen.getByRole('button', { name: '开始导航' })
    await waitFor(() => expect(startButton).toBeEnabled())
    await user.click(startButton)
    expect(JSON.parse(navSocket?.sent.at(-1) ?? '{}')).toMatchObject({
      type: 'navigate',
      action: 'NavigateToPose',
    })
  })

  test('shows navigation and Piper RGB-D as four simultaneous streams', async () => {
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket
    render(<App />)

    await waitFor(() => expect(MockWebSocket.instances).toHaveLength(5))
    expect(screen.getByLabelText('导航 RGB')).toBeInTheDocument()
    expect(screen.getByLabelText('导航深度')).toBeInTheDocument()
    const piperRgbTile = screen.getByLabelText('Piper RGB')
    expect(piperRgbTile).toBeInTheDocument()
    const piperDepthTile = screen.getByLabelText('Piper 深度')
    expect(piperDepthTile).toBeInTheDocument()

    const navDepthSocket = MockWebSocket.instances.find((socket) => socket.url === defaultNavDepthWsUrl())
    const piperRgbSocket = MockWebSocket.instances.find((socket) => socket.url === defaultPiperRgbWsUrl())
    const piperDepthSocket = MockWebSocket.instances.find((socket) => socket.url === defaultPiperDepthWsUrl())
    act(() => {
      navDepthSocket?.onmessage?.({
        data: JSON.stringify({
          image: 'data:image/jpeg;base64,/9j/nav-depth',
          width: 640,
          height: 480,
          topic: '/nav_camera/depth/image_raw',
        }),
      } as MessageEvent)
      piperRgbSocket?.onmessage?.({
        data: JSON.stringify({
          image: 'data:image/jpeg;base64,/9j/piper-rgb',
          width: 320,
          height: 240,
          topic: '/piper/arm_camera/color/image_raw',
        }),
      } as MessageEvent)
      piperDepthSocket?.onmessage?.({
        data: JSON.stringify({
          image: 'data:image/jpeg;base64,/9j/piper-depth',
          width: 320,
          height: 240,
          topic: '/piper/arm_camera/depth/image_raw',
        }),
      } as MessageEvent)
    })

    expect(await screen.findByAltText('导航深度实时画面')).toBeInTheDocument()
    expect(await screen.findByAltText('Piper RGB实时画面')).toBeInTheDocument()
    expect(await screen.findByAltText('Piper 深度实时画面')).toBeInTheDocument()
    expect(screen.getByLabelText('导航深度着色范围 0.25 米到 5 米')).toBeInTheDocument()
    expect(screen.getByLabelText('Piper 深度着色范围 0.15 米到 2.5 米')).toBeInTheDocument()
    expect(within(piperDepthTile).getByRole('button', { name: '截图Piper 深度' })).toBeEnabled()
    expect(within(piperRgbTile).getByRole('button', { name: '截图Piper RGB' })).toBeEnabled()
  })

  test('disconnects and reconnects all visual streams together', async () => {
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket
    const user = userEvent.setup()
    render(<App />)

    await waitFor(() => expect(MockWebSocket.instances).toHaveLength(5))
    const initialSockets = [...MockWebSocket.instances]

    await user.click(screen.getByRole('button', { name: '断开' }))

    expect(initialSockets.every((socket) => socket.readyState === WebSocket.CLOSED)).toBe(true)
    expect(screen.getAllByText('已断开')).toHaveLength(3)

    await user.click(screen.getByRole('button', { name: '重连' }))

    await waitFor(() => expect(MockWebSocket.instances).toHaveLength(10))
    expect(MockWebSocket.instances.filter((socket) => socket.url === defaultNavDepthWsUrl())).toHaveLength(2)
    expect(MockWebSocket.instances.filter((socket) => socket.url === defaultPiperRgbWsUrl())).toHaveLength(2)
    expect(MockWebSocket.instances.filter((socket) => socket.url === defaultPiperDepthWsUrl())).toHaveLength(2)
  })

  test('records received RGB frames and downloads a video file', async () => {
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket
    const stopTrack = vi.fn()
    const fakeStream = {
      getTracks: () => [{ stop: stopTrack }],
    } as unknown as MediaStream
    const createObjectUrl = vi.fn(() => 'blob:slam-nav-recording')
    const revokeObjectUrl = vi.fn()
    const downloadClick = vi.spyOn(HTMLAnchorElement.prototype, 'click').mockImplementation(() => undefined)

    class MockMediaRecorder {
      static isTypeSupported(type: string) {
        return type.startsWith('video/webm')
      }

      state: RecordingState = 'inactive'
      mimeType: string
      ondataavailable: ((event: BlobEvent) => void) | null = null
      onerror: (() => void) | null = null
      onstop: (() => void) | null = null

      constructor(_stream: MediaStream, options?: MediaRecorderOptions) {
        this.mimeType = options?.mimeType ?? 'video/webm'
      }

      start() {
        this.state = 'recording'
      }

      stop() {
        this.state = 'inactive'
        this.ondataavailable?.({ data: new Blob(['recorded-frame']) } as BlobEvent)
        this.onstop?.()
      }
    }

    Object.defineProperty(globalThis, 'MediaRecorder', {
      configurable: true,
      writable: true,
      value: MockMediaRecorder,
    })
    Object.defineProperty(HTMLCanvasElement.prototype, 'captureStream', {
      configurable: true,
      writable: true,
      value: vi.fn(() => fakeStream),
    })
    Object.defineProperty(URL, 'createObjectURL', {
      configurable: true,
      writable: true,
      value: createObjectUrl,
    })
    Object.defineProperty(URL, 'revokeObjectURL', {
      configurable: true,
      writable: true,
      value: revokeObjectUrl,
    })

    const user = userEvent.setup()
    render(<App />)
    const recordButton = screen.getByRole('button', { name: '开始录制' })
    expect(recordButton).toBeDisabled()

    const cameraSocket = MockWebSocket.instances.find((socket) => socket.url === defaultRgbWsUrl())
    act(() => {
      cameraSocket?.onmessage?.({
        data: JSON.stringify({
          image: 'data:image/jpeg;base64,/9j/frame',
          width: 640,
          height: 480,
          fps: 20,
        }),
      } as MessageEvent)
    })

    await waitFor(() => expect(recordButton).toBeEnabled())
    await user.click(recordButton)
    expect(screen.getByRole('button', { name: '停止录制' })).toBeEnabled()

    await user.click(screen.getByRole('button', { name: '停止录制' }))

    expect(createObjectUrl).toHaveBeenCalledWith(expect.any(Blob))
    expect(downloadClick).toHaveBeenCalledOnce()
    expect(stopTrack).toHaveBeenCalledOnce()
    expect(screen.getByText(/录像已导出/)).toBeInTheDocument()
  })

  test('opens the unified task drawer and keeps process logs separate from navigation events', async () => {
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket
    const user = userEvent.setup()
    render(<App />)

    await user.click(screen.getByRole('button', { name: /任务控制/ }))

    expect(screen.getByRole('dialog', { name: '任务控制' })).toBeInTheDocument()
    expect(await screen.findByRole('heading', { name: '可用流程' })).toBeInTheDocument()
    expect(await screen.findByText('自主导航')).toBeInTheDocument()
    expect(screen.getByText('检查').closest('details')).not.toHaveAttribute('open')
    expect(screen.getByLabelText('导航事件日志')).toBeInTheDocument()
    await waitFor(() => {
      expect(screen.getByLabelText('流程日志输出')).toHaveTextContent('[slam_nav_ws] flow output')
    })

    await user.click(screen.getByRole('button', { name: '停止自动建图' }))
    await waitFor(() => {
      const stopRequest = fetchMock.mock.calls.find(([path]) => path === '/api/stop')
      expect(JSON.parse(String(stopRequest?.[1]?.body))).toEqual({ runId: 'run-mapping' })
    })

    await user.click(screen.getByRole('button', { name: '关闭任务控制' }))
    expect(screen.queryByRole('dialog', { name: '任务控制' })).not.toBeInTheDocument()
  })

  test('launches the professional interface through the operator flow and prevents duplicates', async () => {
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket
    const user = userEvent.setup()
    render(<App />)

    const operatorButton = screen.getByRole('button', { name: '打开专业界面' })
    await waitFor(() => expect(operatorButton).toBeEnabled())
    await user.click(operatorButton)

    await waitFor(() => {
      const runRequest = fetchMock.mock.calls.find(([path]) => path === '/api/run')
      expect(JSON.parse(String(runRequest?.[1]?.body))).toEqual({ flowId: 'operator', args: [] })
    })
    await waitFor(() => expect(operatorButton).toBeDisabled())
    expect(operatorButton).toHaveAttribute('aria-label', '专业界面已运行')
  })

  test('shows the operator availability reason and opens tasks from the panel query', async () => {
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket
    taskState = {
      ...taskState,
      operator: { available: false, running: false, reason: '未检测到图形显示环境' },
    }
    window.history.replaceState({}, '', '/?panel=tasks')

    render(<App />)

    expect(screen.getByRole('dialog', { name: '任务控制' })).toBeInTheDocument()
    const operatorButtons = await screen.findAllByRole('button', { name: '打开专业界面' })
    expect(operatorButtons).toHaveLength(2)
    await waitFor(() => {
      operatorButtons.forEach((button) => {
        expect(button).toBeDisabled()
        expect(button).toHaveAttribute('title', '未检测到图形显示环境')
      })
    })
  })
})
