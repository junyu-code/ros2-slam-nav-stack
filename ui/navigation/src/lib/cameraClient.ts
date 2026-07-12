export type CameraFramePayload = {
  image?: string
  imageUrl?: string
  data?: string
  timestamp?: number
  width?: number
  height?: number
  fps?: number
  latencyMs?: number
  topic?: string
}

export type CameraFrame = {
  imageUrl: string
  receivedAt: number
  timestamp?: number
  width?: number
  height?: number
  fps?: number
  latencyMs?: number
  topic?: string
}

export type CameraClientEvent =
  | { type: 'connecting' }
  | { type: 'connected' }
  | { type: 'frame'; frame: CameraFrame }
  | { type: 'error'; message: string }
  | { type: 'disconnected'; code?: number; reason?: string }

type CameraClientOptions = {
  WebSocketImpl?: typeof WebSocket
  now?: () => number
}

type CameraClientListener = (event: CameraClientEvent) => void

const IMAGE_KEYS = ['image', 'imageUrl', 'data'] as const

export function parseCameraFrame(
  payload: CameraFramePayload,
  receivedAt = Date.now(),
): CameraFrame {
  const image = IMAGE_KEYS.map((key) => payload[key]).find(
    (value): value is string => typeof value === 'string' && value.length > 0,
  )

  if (!image) {
    throw new Error('missing image')
  }

  return {
    imageUrl: normalizeImageUrl(image),
    receivedAt,
    timestamp: payload.timestamp,
    width: payload.width,
    height: payload.height,
    fps: payload.fps,
    latencyMs: payload.latencyMs,
    topic: payload.topic,
  }
}

export class CameraClient {
  private socket: WebSocket | null = null
  private listeners = new Set<CameraClientListener>()
  private readonly WebSocketImpl: typeof WebSocket
  private readonly now: () => number
  private readonly url: string

  constructor(url: string, options: CameraClientOptions = {}) {
    this.url = url
    this.WebSocketImpl = options.WebSocketImpl ?? globalThis.WebSocket
    this.now = options.now ?? Date.now
  }

  subscribe(listener: CameraClientListener) {
    this.listeners.add(listener)
    return () => {
      this.listeners.delete(listener)
    }
  }

  connect() {
    if (this.socket) {
      return
    }

    this.emit({ type: 'connecting' })
    const socket = new this.WebSocketImpl(this.url)
    this.socket = socket

    socket.onopen = () => this.emit({ type: 'connected' })
    socket.onerror = () => this.emit({ type: 'error', message: '后端连接异常' })
    socket.onclose = (event) => {
      this.socket = null
      this.emit({
        type: 'disconnected',
        code: event.code,
        reason: event.reason,
      })
    }
    socket.onmessage = (event) => {
      this.handleMessage(event.data)
    }
  }

  disconnect() {
    this.socket?.close()
    this.socket = null
  }

  private handleMessage(data: unknown) {
    try {
      const payload = parsePayload(data)
      this.emit({ type: 'frame', frame: parseCameraFrame(payload, this.now()) })
    } catch (error) {
      this.emit({
        type: 'error',
        message: error instanceof Error ? error.message : '图像帧解析失败',
      })
    }
  }

  private emit(event: CameraClientEvent) {
    for (const listener of this.listeners) {
      listener(event)
    }
  }
}

function parsePayload(data: unknown): CameraFramePayload {
  if (typeof data === 'string') {
    return JSON.parse(data) as CameraFramePayload
  }

  throw new Error('unsupported frame payload')
}

function normalizeImageUrl(image: string) {
  if (image.startsWith('data:') || image.startsWith('blob:')) {
    return image
  }

  return `data:image/jpeg;base64,${image}`
}

