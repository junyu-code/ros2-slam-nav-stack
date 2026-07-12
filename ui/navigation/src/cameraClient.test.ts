import { afterEach, describe, expect, test, vi } from 'vitest'
import {
  CameraClient,
  parseCameraFrame,
  type CameraFramePayload,
} from './lib/cameraClient'

const originalWebSocket = globalThis.WebSocket

class MockWebSocket {
  static instances: MockWebSocket[] = []

  onopen: (() => void) | null = null
  onclose: ((event: CloseEvent) => void) | null = null
  onerror: (() => void) | null = null
  onmessage: ((event: MessageEvent) => void) | null = null
  sent: string[] = []

  readonly url: string

  constructor(url: string) {
    this.url = url
    MockWebSocket.instances.push(this)
  }

  close() {
    this.onclose?.({ code: 1000, reason: 'client closed' } as CloseEvent)
  }

  send(message: string) {
    this.sent.push(message)
  }
}

afterEach(() => {
  globalThis.WebSocket = originalWebSocket
  MockWebSocket.instances = []
  vi.restoreAllMocks()
})

describe('parseCameraFrame', () => {
  test('normalizes backend frame payloads into renderable image metadata', () => {
    const payload: CameraFramePayload = {
      image: 'data:image/jpeg;base64,/9j/example',
      timestamp: 1_800,
      width: 1280,
      height: 720,
      fps: 29.8,
      latencyMs: 18,
      topic: '/camera/color/image_raw',
    }

    expect(parseCameraFrame(payload, 2_000)).toEqual({
      imageUrl: 'data:image/jpeg;base64,/9j/example',
      receivedAt: 2_000,
      timestamp: 1_800,
      width: 1280,
      height: 720,
      fps: 29.8,
      latencyMs: 18,
      topic: '/camera/color/image_raw',
    })
  })

  test('rejects payloads without a renderable JPEG image', () => {
    expect(() =>
      parseCameraFrame({ image: '', timestamp: 1 } as CameraFramePayload, 2),
    ).toThrow('missing image')
  })
})

describe('CameraClient', () => {
  test('opens a WebSocket and emits connected, frame, and disconnected events', () => {
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket
    const events: string[] = []
    const frames: string[] = []
    const client = new CameraClient('ws://127.0.0.1:8765/ws/rgb', {
      now: () => 2_500,
    })

    client.subscribe((event) => {
      events.push(event.type)
      if (event.type === 'frame') {
        frames.push(event.frame.imageUrl)
      }
    })

    client.connect()
    const socket = MockWebSocket.instances[0]
    socket.onopen?.()
    socket.onmessage?.({
      data: JSON.stringify({
        image: 'data:image/jpeg;base64,/9j/frame',
        timestamp: 2_450,
        width: 640,
        height: 480,
      }),
    } as MessageEvent)
    client.disconnect()

    expect(socket.url).toBe('ws://127.0.0.1:8765/ws/rgb')
    expect(events).toEqual(['connecting', 'connected', 'frame', 'disconnected'])
    expect(frames).toEqual(['data:image/jpeg;base64,/9j/frame'])
  })
})

