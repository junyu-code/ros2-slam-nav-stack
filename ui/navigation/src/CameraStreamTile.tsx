import { Activity, Camera, Expand, RefreshCw, Save, ScanLine } from 'lucide-react'
import { useEffect, useMemo, useRef, useState } from 'react'
import { CameraClient, type CameraFrame } from './lib/cameraClient'

type StreamStatus = 'idle' | 'connecting' | 'connected' | 'error'

type CameraStreamTileProps = {
  label: string
  topic: string
  wsPath: string
  configuredUrl?: string
  enabled?: boolean
  reconnectSignal?: number
  depth?: boolean
  minDepthM?: number
  maxDepthM?: number
}

const RECONNECT_DELAY_MS = 1_500
const STALE_FRAME_MS = 3_000

function defaultWebSocketUrl(path: string, configured?: string) {
  if (configured) {
    return configured
  }
  if (typeof window !== 'undefined' && window.location.host) {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
    return `${protocol}//${window.location.host}${path}`
  }
  return `ws://127.0.0.1:8765${path}`
}

export function CameraStreamTile({
  label,
  topic,
  wsPath,
  configuredUrl,
  enabled = true,
  reconnectSignal = 0,
  depth = false,
  minDepthM = 0.15,
  maxDepthM = 2.5,
}: CameraStreamTileProps) {
  const [status, setStatus] = useState<StreamStatus>('connecting')
  const [frame, setFrame] = useState<CameraFrame | null>(null)
  const [connectionVersion, setConnectionVersion] = useState(0)
  const [clock, setClock] = useState(() => Date.now())
  const viewportRef = useRef<HTMLDivElement | null>(null)
  const url = useMemo(
    () => defaultWebSocketUrl(wsPath, configuredUrl),
    [configuredUrl, wsPath],
  )

  useEffect(() => {
    if (!enabled) {
      setFrame(null)
      setStatus('idle')
      return
    }

    let disposed = false
    let reconnectTimer: number | null = null
    let client: CameraClient | null = null

    const connect = () => {
      if (disposed) {
        return
      }
      setStatus('connecting')
      client = new CameraClient(url)
      client.subscribe((event) => {
        if (disposed) {
          return
        }
        if (event.type === 'connected') {
          setStatus('connected')
        }
        if (event.type === 'frame') {
          setFrame(event.frame)
          setStatus('connected')
        }
        if (event.type === 'error') {
          setStatus('error')
        }
        if (event.type === 'disconnected') {
          setStatus('connecting')
          reconnectTimer = window.setTimeout(connect, RECONNECT_DELAY_MS)
        }
      })
      client.connect()
    }

    setFrame(null)
    connect()
    return () => {
      disposed = true
      if (reconnectTimer !== null) {
        window.clearTimeout(reconnectTimer)
      }
      client?.disconnect()
    }
  }, [connectionVersion, enabled, reconnectSignal, url])

  useEffect(() => {
    const timer = window.setInterval(() => setClock(Date.now()), 1_000)
    return () => window.clearInterval(timer)
  }, [])

  const frameAgeMs = frame ? Math.max(0, clock - frame.receivedAt) : null
  const stale = status === 'connected' && frameAgeMs !== null && frameAgeMs >= STALE_FRAME_MS

  function reconnect() {
    setConnectionVersion((current) => current + 1)
  }

  function requestFullscreen() {
    void viewportRef.current?.requestFullscreen?.()
  }

  function captureSnapshot() {
    if (!frame) {
      return
    }
    const anchor = document.createElement('a')
    anchor.href = frame.imageUrl
    anchor.download = `slam-nav-${label}-${new Date().toISOString().replace(/[:.]/g, '-')}.jpg`
    anchor.click()
  }

  return (
    <article className="vision-tile" aria-label={label}>
      <header className="vision-tile-header">
        <div>
          {depth ? <ScanLine size={14} /> : <Camera size={14} />}
          <strong>{label}</strong>
        </div>
        <div className="vision-tile-actions">
          <button
            type="button"
            onClick={reconnect}
            disabled={!enabled}
            aria-label={`重连${label}`}
            title={`重连${label}`}
          >
            <RefreshCw size={13} />
          </button>
          <button
            type="button"
            onClick={captureSnapshot}
            disabled={!frame}
            aria-label={`截图${label}`}
            title={`截图${label}`}
          >
            <Save size={13} />
          </button>
          <button type="button" onClick={requestFullscreen} aria-label={`全屏${label}`} title={`全屏${label}`}>
            <Expand size={13} />
          </button>
        </div>
      </header>

      <div className="vision-viewport" ref={viewportRef}>
        {frame ? (
          <img className="video-frame" src={frame.imageUrl} alt={`${label}实时画面`} />
        ) : (
          <div className="empty-state compact-empty-state">
            {depth ? <ScanLine size={20} /> : <Activity size={20} />}
            <strong>{status === 'error' ? '图像桥异常' : `等待 ${topic}`}</strong>
          </div>
        )}
        <span className={`stream-status ${status}${stale ? ' stale' : ''}`}>
          {stale
            ? '画面停滞'
            : status === 'connected'
              ? '已连接'
              : status === 'error'
                ? '异常'
                : status === 'idle'
                  ? '已断开'
                  : '连接中'}
        </span>
        {depth ? (
          <div className="depth-scale" aria-label={`${label}着色范围 ${minDepthM} 米到 ${maxDepthM} 米`}>
            <span>{minDepthM} m</span>
            <i />
            <span>{maxDepthM} m</span>
          </div>
        ) : null}
      </div>

      <footer className="vision-meta">
        <span>{formatNumber(frame?.fps)} FPS</span>
        <span>{formatResolution(frame)}</span>
        <span>{formatFrameAge(frameAgeMs)}</span>
      </footer>
    </article>
  )
}

function formatNumber(value?: number) {
  return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(1) : '--'
}

function formatResolution(frame: CameraFrame | null) {
  if (!frame?.width || !frame.height) {
    return '--'
  }
  return `${frame.width}x${frame.height}`
}

function formatFrameAge(ageMs: number | null) {
  if (ageMs === null) {
    return '--'
  }
  if (ageMs < 1_000) {
    return `${Math.round(ageMs)} ms`
  }
  return `${(ageMs / 1_000).toFixed(1)} s`
}
