export type MapOrigin = {
  x: number
  y: number
  yaw: number
}

export type MapMessage = {
  type: 'map'
  width: number
  height: number
  resolution: number
  origin: MapOrigin
  data: number[]
  frame?: string
  topic?: string
}

export type CostmapCell = {
  x: number
  y: number
  value: number
}

export type CostmapMessage = {
  type: 'costmap'
  scope: 'global' | 'local'
  width: number
  height: number
  resolution: number
  origin: MapOrigin
  cells: CostmapCell[]
  points: CostmapCell[]
  frame?: string
  sourceFrame?: string
  topic?: string
}

export type PathPoint = {
  x: number
  y: number
}

export type PathMessage = {
  type: 'path'
  scope: 'global' | 'local'
  points: PathPoint[]
  frame?: string
  sourceFrame?: string
  topic?: string
}

export type RobotPoseMessage = {
  type: 'robot_pose'
  x: number
  y: number
  yaw: number
  frame?: string
  sourceFrame?: string
}

export type NavStatusMessage = {
  type: 'nav_status'
  state: 'waiting' | 'ready' | 'sent' | 'executing' | 'succeeded' | 'canceled' | 'failed' | 'error'
  detail?: string
  action?: NavigationAction
}

export type NavigationReadyMessage = {
  type: 'navigation_ready'
  ready: boolean
  topic?: string
}

export type NavigationMessage =
  | MapMessage
  | CostmapMessage
  | PathMessage
  | RobotPoseMessage
  | NavigationReadyMessage
  | NavStatusMessage

export type WaypointDraft = {
  x: number
  y: number
  yaw?: number
}

export type Waypoint = {
  x: number
  y: number
  yaw: number
}

export type NavigationAction = 'NavigateToPose' | 'NavigateThroughPoses'

export type NavigateCommand = {
  type: 'navigate'
  action: NavigationAction
  waypoints: Waypoint[]
}

export type NavigationClientEvent =
  | { type: 'connecting' }
  | { type: 'connected' }
  | { type: 'message'; message: NavigationMessage }
  | { type: 'error'; message: string }
  | { type: 'disconnected'; code?: number; reason?: string }

type NavigationClientOptions = {
  WebSocketImpl?: typeof WebSocket
}

type NavigationClientListener = (event: NavigationClientEvent) => void

export function parseNavigationMessage(data: string): NavigationMessage {
  const payload = JSON.parse(data) as unknown
  if (!isRecord(payload) || typeof payload.type !== 'string') {
    throw new Error('unsupported navigation message')
  }

  switch (payload.type) {
    case 'map':
      return parseMapMessage(payload)
    case 'costmap':
      return parseCostmapMessage(payload)
    case 'path':
      return parsePathMessage(payload)
    case 'robot_pose':
      return parseRobotPoseMessage(payload)
    case 'navigation_ready':
      return parseNavigationReadyMessage(payload)
    case 'nav_status':
      return parseNavStatusMessage(payload)
    default:
      throw new Error('unsupported navigation message')
  }
}

export function resolveWaypointOrientations(waypoints: WaypointDraft[], fallbackYaw = 0): Waypoint[] {
  return waypoints.map((waypoint, index) => {
    if (typeof waypoint.yaw === 'number') {
      return { x: waypoint.x, y: waypoint.y, yaw: waypoint.yaw }
    }

    const next = waypoints[index + 1]
    const previous = waypoints[index - 1]
    let yaw = fallbackYaw
    if (next) {
      yaw = Math.atan2(next.y - waypoint.y, next.x - waypoint.x)
    } else if (previous) {
      yaw = Math.atan2(waypoint.y - previous.y, waypoint.x - previous.x)
    }

    return { x: waypoint.x, y: waypoint.y, yaw }
  })
}

export function createNavigateCommand(waypoints: WaypointDraft[]): NavigateCommand {
  const resolved = resolveWaypointOrientations(waypoints)
  return {
    type: 'navigate',
    action: resolved.length > 1 ? 'NavigateThroughPoses' : 'NavigateToPose',
    waypoints: resolved,
  }
}

export class NavigationClient {
  private socket: WebSocket | null = null
  private listeners = new Set<NavigationClientListener>()
  private readonly WebSocketImpl: typeof WebSocket
  private readonly url: string

  constructor(url: string, options: NavigationClientOptions = {}) {
    this.url = url
    this.WebSocketImpl = options.WebSocketImpl ?? globalThis.WebSocket
  }

  subscribe(listener: NavigationClientListener) {
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
    socket.onerror = () => this.emit({ type: 'error', message: '导航桥连接异常' })
    socket.onclose = (event) => {
      this.socket = null
      this.emit({ type: 'disconnected', code: event.code, reason: event.reason })
    }
    socket.onmessage = (event) => {
      try {
        this.emit({ type: 'message', message: parseNavigationMessage(String(event.data)) })
      } catch (error) {
        this.emit({
          type: 'error',
          message: error instanceof Error ? error.message : '导航消息解析失败',
        })
      }
    }
  }

  send(command: NavigateCommand | { type: 'cancel_navigation' } | { type: 'ping' }) {
    if (this.socket?.readyState !== WebSocket.OPEN) {
      throw new Error('navigation websocket is not connected')
    }
    this.socket.send(JSON.stringify(command))
  }

  disconnect() {
    this.socket?.close()
    this.socket = null
  }

  private emit(event: NavigationClientEvent) {
    for (const listener of this.listeners) {
      listener(event)
    }
  }
}

function parseMapMessage(payload: Record<string, unknown>): MapMessage {
  return {
    type: 'map',
    width: asNumber(payload.width),
    height: asNumber(payload.height),
    resolution: asNumber(payload.resolution),
    origin: asOrigin(payload.origin),
    data: asNumberArray(payload.data),
    frame: asOptionalString(payload.frame),
    topic: asOptionalString(payload.topic),
  }
}

function parseCostmapMessage(payload: Record<string, unknown>): CostmapMessage {
  const scope = payload.scope === 'local' ? 'local' : 'global'
  return {
    type: 'costmap',
    scope,
    width: asNumber(payload.width),
    height: asNumber(payload.height),
    resolution: asNumber(payload.resolution),
    origin: asOrigin(payload.origin),
    cells: Array.isArray(payload.cells)
      ? payload.cells.filter(isRecord).map((cell) => ({
          x: asNumber(cell.x),
          y: asNumber(cell.y),
          value: asNumber(cell.value),
        }))
      : [],
    points: Array.isArray(payload.points)
      ? payload.points.filter(isRecord).map((point) => ({
          x: asNumber(point.x),
          y: asNumber(point.y),
          value: asNumber(point.value),
        }))
      : [],
    frame: asOptionalString(payload.frame),
    sourceFrame: asOptionalString(payload.sourceFrame),
    topic: asOptionalString(payload.topic),
  }
}

function parsePathMessage(payload: Record<string, unknown>): PathMessage {
  const scope = payload.scope === 'local' ? 'local' : 'global'
  return {
    type: 'path',
    scope,
    points: Array.isArray(payload.points)
      ? payload.points.filter(isRecord).map((point) => ({
          x: asNumber(point.x),
          y: asNumber(point.y),
        }))
      : [],
    frame: asOptionalString(payload.frame),
    sourceFrame: asOptionalString(payload.sourceFrame),
    topic: asOptionalString(payload.topic),
  }
}

function parseRobotPoseMessage(payload: Record<string, unknown>): RobotPoseMessage {
  return {
    type: 'robot_pose',
    x: asNumber(payload.x),
    y: asNumber(payload.y),
    yaw: asNumber(payload.yaw),
    frame: asOptionalString(payload.frame),
    sourceFrame: asOptionalString(payload.sourceFrame),
  }
}

function parseNavStatusMessage(payload: Record<string, unknown>): NavStatusMessage {
  const state = typeof payload.state === 'string' ? payload.state : 'error'
  return {
    type: 'nav_status',
    state: isNavState(state) ? state : 'error',
    detail: asOptionalString(payload.detail),
    action: isNavigationAction(payload.action) ? payload.action : undefined,
  }
}

function parseNavigationReadyMessage(payload: Record<string, unknown>): NavigationReadyMessage {
  if (typeof payload.ready !== 'boolean') {
    throw new Error('invalid navigation ready message')
  }
  return {
    type: 'navigation_ready',
    ready: payload.ready,
    topic: asOptionalString(payload.topic),
  }
}

function asOrigin(value: unknown): MapOrigin {
  if (!isRecord(value)) {
    return { x: 0, y: 0, yaw: 0 }
  }
  return {
    x: asNumber(value.x),
    y: asNumber(value.y),
    yaw: asNumber(value.yaw),
  }
}

function asNumber(value: unknown): number {
  return typeof value === 'number' && Number.isFinite(value) ? value : 0
}

function asNumberArray(value: unknown): number[] {
  return Array.isArray(value) ? value.map(asNumber) : []
}

function asOptionalString(value: unknown): string | undefined {
  return typeof value === 'string' ? value : undefined
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null
}

function isNavigationAction(value: unknown): value is NavigationAction {
  return value === 'NavigateToPose' || value === 'NavigateThroughPoses'
}

function isNavState(value: string): value is NavStatusMessage['state'] {
  return ['waiting', 'ready', 'sent', 'executing', 'succeeded', 'canceled', 'failed', 'error'].includes(value)
}
