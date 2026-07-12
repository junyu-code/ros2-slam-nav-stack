import {
  Activity,
  Camera,
  CircleDot,
  Expand,
  Flag,
  ListTodo,
  LocateFixed,
  Map,
  MonitorUp,
  Navigation,
  Pause,
  Play,
  Radio,
  RotateCcw,
  Route,
  Save,
  Send,
  Square,
  Trash2,
  Video,
  WifiOff,
  X,
} from 'lucide-react'
import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import './App.css'
import { CameraStreamTile } from './CameraStreamTile'
import { TaskDrawer } from './TaskDrawer'
import { CameraClient, type CameraFrame } from './lib/cameraClient'
import {
  canvasToWorld,
  createMapTransform,
  gridCellToWorld,
  worldToCanvas,
  type OccupancyGridGeometry,
} from './lib/mapGeometry'
import {
  createNavigateCommand,
  type NavigationMessage,
  NavigationClient,
  type CostmapMessage,
  type MapMessage,
  type NavStatusMessage,
  type PathMessage,
  type RobotPoseMessage,
  type WaypointDraft,
} from './lib/navClient'
import { useTaskControl } from './lib/useTaskControl'

type ConnectionStatus = 'idle' | 'connecting' | 'connected' | 'error' | 'timeout'
type NavigationReadinessState = 'unknown' | 'available' | 'unavailable'

type LogEntry = {
  id: number
  level: 'info' | 'error'
  message: string
  time: string
}

type CostmapLayers = {
  global: CostmapMessage | null
  local: CostmapMessage | null
}

type PathLayers = {
  global: PathMessage | null
  local: PathMessage | null
}

const DEFAULT_CAMERA_URL = getDefaultWebSocketUrl('/ws/rgb', 'VITE_CAMERA_WS_URL')
const DEFAULT_NAV_URL = getDefaultWebSocketUrl('/ws/nav', 'VITE_NAV_WS_URL')
const DEFAULT_CAMERA_TOPIC =
  import.meta.env.VITE_CAMERA_TOPIC ?? '/nav_camera/color/image_raw'
const DEFAULT_NAV_DEPTH_TOPIC = '/nav_camera/depth/image_raw'
const DEFAULT_MAP_TOPIC = import.meta.env.VITE_MAP_TOPIC ?? '/map'
const RECONNECT_DELAY_MS = 1_500

const STATUS_LABELS: Record<ConnectionStatus, string> = {
  idle: '等待连接',
  connecting: '连接中',
  connected: '已连接',
  error: '连接异常',
  timeout: '帧超时',
}

const NAV_STATE_LABELS: Record<NavStatusMessage['state'], string> = {
  waiting: '等待',
  ready: '就绪',
  sent: '已发送',
  executing: '执行中',
  succeeded: '已完成',
  canceled: '已取消',
  failed: '失败',
  error: '异常',
}

const NAVIGATION_READINESS_LABELS: Record<NavigationReadinessState, string> = {
  unknown: '检测中',
  available: '可以开始',
  unavailable: '未就绪',
}

function getDefaultWebSocketUrl(path: string, envKey: 'VITE_CAMERA_WS_URL' | 'VITE_NAV_WS_URL') {
  const configuredUrl = import.meta.env[envKey]
  if (configuredUrl) {
    return configuredUrl
  }

  if (typeof window !== 'undefined' && window.location.host) {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
    return `${protocol}//${window.location.host}${path}`
  }

  return `ws://127.0.0.1:8765${path}`
}

function App() {
  const [taskDrawerOpen, setTaskDrawerOpen] = useState(
    () => typeof window !== 'undefined' && new URLSearchParams(window.location.search).get('panel') === 'tasks',
  )
  const [cameraUrl, setCameraUrl] = useState(DEFAULT_CAMERA_URL)
  const [navUrl, setNavUrl] = useState(DEFAULT_NAV_URL)
  const [cameraStatus, setCameraStatus] = useState<ConnectionStatus>('idle')
  const [navConnectionStatus, setNavConnectionStatus] = useState<ConnectionStatus>('idle')
  const [navStatus, setNavStatus] = useState<NavStatusMessage>({
    type: 'nav_status',
    state: 'waiting',
    detail: '等待导航桥连接',
  })
  const [navigationReady, setNavigationReady] = useState<boolean | null>(null)
  const [frame, setFrame] = useState<CameraFrame | null>(null)
  const [auxiliaryStreamsEnabled, setAuxiliaryStreamsEnabled] = useState(true)
  const [auxiliaryReconnectSignal, setAuxiliaryReconnectSignal] = useState(0)
  const [paused, setPaused] = useState(false)
  const [recording, setRecording] = useState(false)
  const [mapData, setMapData] = useState<MapMessage | null>(null)
  const [costmaps, setCostmaps] = useState<CostmapLayers>({ global: null, local: null })
  const [paths, setPaths] = useState<PathLayers>({ global: null, local: null })
  const [robotPose, setRobotPose] = useState<RobotPoseMessage | null>(null)
  const [waypoints, setWaypoints] = useState<WaypointDraft[]>([])
  const [logs, setLogs] = useState<LogEntry[]>([
    createLog('info', '页面加载后会自动连接相机桥和导航桥'),
  ])

  const cameraClientRef = useRef<CameraClient | null>(null)
  const navClientRef = useRef<NavigationClient | null>(null)
  const imageRef = useRef<HTMLImageElement | null>(null)
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const cameraUrlRef = useRef(DEFAULT_CAMERA_URL)
  const navUrlRef = useRef(DEFAULT_NAV_URL)
  const pausedRef = useRef(false)
  const manualCameraDisconnectRef = useRef(false)
  const manualNavDisconnectRef = useRef(false)
  const navigationReadyRef = useRef<boolean | null>(null)
  const suppressCameraCloseRef = useRef(false)
  const suppressNavCloseRef = useRef(false)
  const cameraReconnectTimerRef = useRef<number | null>(null)
  const navReconnectTimerRef = useRef<number | null>(null)
  const mediaRecorderRef = useRef<MediaRecorder | null>(null)
  const recordingCanvasRef = useRef<HTMLCanvasElement | null>(null)
  const recordingStreamRef = useRef<MediaStream | null>(null)
  const recordingChunksRef = useRef<Blob[]>([])
  const taskControl = useTaskControl()

  const operatorRunning = Boolean(
    taskControl.state?.operator?.running ||
      taskControl.state?.active.some((run) => run.flowId === 'operator'),
  )
  const operatorAvailable = Boolean(taskControl.state?.operator?.available)
  const operatorButtonTitle =
    taskControl.state?.operator?.reason ??
    (operatorRunning ? '专业界面已运行' : operatorAvailable ? '打开 Qt / RViz Operator' : '正在检查专业界面状态')

  const mapGeometry = useMemo<OccupancyGridGeometry | null>(() => {
    if (!mapData) {
      return null
    }
    return {
      width: mapData.width,
      height: mapData.height,
      resolution: mapData.resolution,
      origin: mapData.origin,
    }
  }, [mapData])

  useEffect(() => {
    cameraUrlRef.current = cameraUrl
  }, [cameraUrl])

  useEffect(() => {
    navUrlRef.current = navUrl
  }, [navUrl])

  useEffect(() => {
    pausedRef.current = paused
  }, [paused])

  useEffect(() => {
    startCameraConnection(DEFAULT_CAMERA_URL, 'auto')
    startNavConnection(DEFAULT_NAV_URL, 'auto')

    return () => {
      manualCameraDisconnectRef.current = true
      manualNavDisconnectRef.current = true
      clearCameraReconnectTimer()
      clearNavReconnectTimer()
      cameraClientRef.current?.disconnect()
      navClientRef.current?.disconnect()
      cameraClientRef.current = null
      navClientRef.current = null
      const recorder = mediaRecorderRef.current
      if (recorder && recorder.state !== 'inactive') {
        recorder.ondataavailable = null
        recorder.onstop = null
        recorder.stop()
      }
      recordingStreamRef.current?.getTracks().forEach((track) => track.stop())
      mediaRecorderRef.current = null
      recordingCanvasRef.current = null
      recordingStreamRef.current = null
      recordingChunksRef.current = []
    }
  }, [])

  function appendLog(level: LogEntry['level'], message: string) {
    setLogs((current) => [createLog(level, message), ...current].slice(0, 9))
  }

  function clearCameraReconnectTimer() {
    if (cameraReconnectTimerRef.current !== null) {
      window.clearTimeout(cameraReconnectTimerRef.current)
      cameraReconnectTimerRef.current = null
    }
  }

  function clearNavReconnectTimer() {
    if (navReconnectTimerRef.current !== null) {
      window.clearTimeout(navReconnectTimerRef.current)
      navReconnectTimerRef.current = null
    }
  }

  function startCameraConnection(url = cameraUrlRef.current, reason: 'auto' | 'manual' = 'manual') {
    const trimmedUrl = url.trim()
    if (!trimmedUrl) {
      setCameraStatus('error')
      appendLog('error', '相机桥地址不能为空')
      return
    }

    clearCameraReconnectTimer()
    if (cameraClientRef.current) {
      suppressCameraCloseRef.current = true
      cameraClientRef.current.disconnect()
      cameraClientRef.current = null
    }

    manualCameraDisconnectRef.current = false
    cameraUrlRef.current = trimmedUrl
    setCameraUrl(trimmedUrl)
    setCameraStatus('connecting')
    appendLog('info', reason === 'auto' ? `自动连接相机桥 ${trimmedUrl}` : `连接相机桥 ${trimmedUrl}`)

    const client = new CameraClient(trimmedUrl)
    cameraClientRef.current = client
    client.subscribe((event) => {
      if (event.type === 'connecting') {
        setCameraStatus('connecting')
      }
      if (event.type === 'connected') {
        setCameraStatus('connected')
        appendLog('info', '相机桥 WebSocket 已连接')
      }
      if (event.type === 'frame') {
        drawRecordingFrame(event.frame)
        if (!pausedRef.current) {
          setFrame(event.frame)
        }
        setCameraStatus('connected')
      }
      if (event.type === 'error') {
        setCameraStatus('error')
        appendLog('error', event.message)
      }
      if (event.type === 'disconnected') {
        cameraClientRef.current = null
        if (suppressCameraCloseRef.current) {
          suppressCameraCloseRef.current = false
          return
        }
        if (manualCameraDisconnectRef.current) {
          setCameraStatus('idle')
          return
        }
        setCameraStatus('connecting')
        appendLog('info', '相机桥中断，等待后端恢复并自动重连')
        cameraReconnectTimerRef.current = window.setTimeout(() => {
          startCameraConnection(cameraUrlRef.current, 'auto')
        }, RECONNECT_DELAY_MS)
      }
    })

    client.connect()
  }

  function startNavConnection(url = navUrlRef.current, reason: 'auto' | 'manual' = 'manual') {
    const trimmedUrl = url.trim()
    if (!trimmedUrl) {
      setNavConnectionStatus('error')
      appendLog('error', '导航桥地址不能为空')
      return
    }

    clearNavReconnectTimer()
    if (navClientRef.current) {
      suppressNavCloseRef.current = true
      navClientRef.current.disconnect()
      navClientRef.current = null
    }

    manualNavDisconnectRef.current = false
    navigationReadyRef.current = null
    setNavigationReady(null)
    navUrlRef.current = trimmedUrl
    setNavUrl(trimmedUrl)
    setNavConnectionStatus('connecting')
    appendLog('info', reason === 'auto' ? `自动连接导航桥 ${trimmedUrl}` : `连接导航桥 ${trimmedUrl}`)

    const client = new NavigationClient(trimmedUrl)
    navClientRef.current = client
    client.subscribe((event) => {
      if (event.type === 'connecting') {
        setNavConnectionStatus('connecting')
      }
      if (event.type === 'connected') {
        setNavConnectionStatus('connected')
        appendLog('info', '导航桥 WebSocket 已连接')
      }
      if (event.type === 'message') {
        setNavConnectionStatus('connected')
        applyNavigationMessage(event.message)
      }
      if (event.type === 'error') {
        setNavConnectionStatus('error')
        appendLog('error', event.message)
      }
      if (event.type === 'disconnected') {
        navClientRef.current = null
        navigationReadyRef.current = null
        setNavigationReady(null)
        if (suppressNavCloseRef.current) {
          suppressNavCloseRef.current = false
          return
        }
        if (manualNavDisconnectRef.current) {
          setNavConnectionStatus('idle')
          return
        }
        setNavConnectionStatus('connecting')
        appendLog('info', '导航桥中断，等待后端恢复并自动重连')
        navReconnectTimerRef.current = window.setTimeout(() => {
          startNavConnection(navUrlRef.current, 'auto')
        }, RECONNECT_DELAY_MS)
      }
    })

    client.connect()
  }

  function applyNavigationMessage(message: NavigationMessage) {
    if (message.type === 'map') {
      setMapData(message)
      return
    }
    if (message.type === 'costmap') {
      setCostmaps((current) => ({ ...current, [message.scope]: message }))
      return
    }
    if (message.type === 'path') {
      setPaths((current) => ({ ...current, [message.scope]: message }))
      return
    }
    if (message.type === 'robot_pose') {
      setRobotPose(message)
      return
    }
    if (message.type === 'navigation_ready') {
      const previous = navigationReadyRef.current
      navigationReadyRef.current = message.ready
      setNavigationReady(message.ready)
      if (previous !== message.ready) {
        appendLog(
          previous === true && !message.ready ? 'error' : 'info',
          message.ready ? '导航已就绪，可以开始导航' : '导航尚未就绪，暂不能下发目标',
        )
      }
      return
    }
    if (message.type === 'nav_status') {
      setNavStatus(message)
      if (message.detail) {
        appendLog(message.state === 'error' || message.state === 'failed' ? 'error' : 'info', message.detail)
      }
    }
  }

  function disconnectAll() {
    manualCameraDisconnectRef.current = true
    manualNavDisconnectRef.current = true
    clearCameraReconnectTimer()
    clearNavReconnectTimer()
    cameraClientRef.current?.disconnect()
    navClientRef.current?.disconnect()
    cameraClientRef.current = null
    navClientRef.current = null
    setAuxiliaryStreamsEnabled(false)
    navigationReadyRef.current = null
    setNavigationReady(null)
    setCameraStatus('idle')
    setNavConnectionStatus('idle')
    appendLog('info', '已主动断开四路视觉流和导航桥')
  }

  function togglePaused() {
    setPaused((current) => {
      const next = !current
      appendLog('info', next ? '暂停相机预览' : '恢复相机预览')
      return next
    })
  }

  function captureSnapshot() {
    if (!imageRef.current?.src) {
      appendLog('error', '当前没有可截图的图像帧')
      return
    }

    const link = document.createElement('a')
    link.href = imageRef.current.src
    link.download = `slam-nav-rgb-${new Date().toISOString().replace(/[:.]/g, '-')}.jpg`
    link.click()
    appendLog('info', '已导出当前 RGB 截图')
  }

  function requestCameraFullscreen() {
    const target = document.querySelector('#nav-rgb-view')
    if (target instanceof HTMLElement) {
      void target.requestFullscreen?.()
    }
  }

  function drawRecordingFrame(nextFrame: CameraFrame) {
    const canvas = recordingCanvasRef.current
    const recorder = mediaRecorderRef.current
    if (!canvas || !recorder || recorder.state === 'inactive') {
      return
    }

    const image = new Image()
    image.onload = () => {
      if (recordingCanvasRef.current !== canvas || mediaRecorderRef.current !== recorder) {
        return
      }
      const width = nextFrame.width || image.naturalWidth
      const height = nextFrame.height || image.naturalHeight
      if (!width || !height) {
        return
      }
      if (canvas.width !== width || canvas.height !== height) {
        canvas.width = width
        canvas.height = height
      }
      canvas.getContext('2d')?.drawImage(image, 0, 0, width, height)
    }
    image.src = nextFrame.imageUrl
  }

  function startRecording() {
    if (!frame) {
      appendLog('error', '当前没有可录制的 RGB 图像帧')
      return
    }
    if (typeof MediaRecorder === 'undefined') {
      appendLog('error', '当前浏览器不支持视频录制')
      return
    }

    const canvas = document.createElement('canvas')
    if (typeof canvas.captureStream !== 'function') {
      appendLog('error', '当前浏览器不支持画布视频流')
      return
    }
    canvas.width = frame.width || imageRef.current?.naturalWidth || 1280
    canvas.height = frame.height || imageRef.current?.naturalHeight || 720

    const frameRate = Math.max(1, Math.min(30, Math.round(frame.fps || 15)))
    const stream = canvas.captureStream(frameRate)
    const mimeType = selectRecordingMimeType()

    try {
      const recorder = mimeType
        ? new MediaRecorder(stream, { mimeType })
        : new MediaRecorder(stream)
      recordingChunksRef.current = []
      recordingCanvasRef.current = canvas
      recordingStreamRef.current = stream
      mediaRecorderRef.current = recorder

      recorder.ondataavailable = (event) => {
        if (event.data.size > 0) {
          recordingChunksRef.current.push(event.data)
        }
      }
      recorder.onerror = () => {
        appendLog('error', '视频编码失败，正在结束本次录制')
        if (recorder.state !== 'inactive') {
          recorder.stop()
        }
      }
      recorder.onstop = () => {
        stream.getTracks().forEach((track) => track.stop())
        const chunks = recordingChunksRef.current
        const outputType = recorder.mimeType || mimeType || 'video/webm'
        mediaRecorderRef.current = null
        recordingCanvasRef.current = null
        recordingStreamRef.current = null
        recordingChunksRef.current = []
        setRecording(false)

        if (!chunks.length) {
          appendLog('error', '录制未产生有效视频数据')
          return
        }

        const blob = new Blob(chunks, { type: outputType })
        const extension = outputType.includes('mp4') ? 'mp4' : 'webm'
        const url = URL.createObjectURL(blob)
        const link = document.createElement('a')
        link.href = url
        link.download = `slam-nav-rgb-${new Date().toISOString().replace(/[:.]/g, '-')}.${extension}`
        link.click()
        window.setTimeout(() => URL.revokeObjectURL(url), 0)
        appendLog('info', `录像已导出：${formatFileSize(blob.size)}`)
      }

      recorder.start(1_000)
      setRecording(true)
      drawRecordingFrame(frame)
      appendLog('info', `开始录制 ${canvas.width} x ${canvas.height} @ ${frameRate} FPS`)
    } catch (error) {
      stream.getTracks().forEach((track) => track.stop())
      mediaRecorderRef.current = null
      recordingCanvasRef.current = null
      recordingStreamRef.current = null
      recordingChunksRef.current = []
      appendLog('error', error instanceof Error ? error.message : '无法启动视频录制')
    }
  }

  function stopRecording() {
    const recorder = mediaRecorderRef.current
    if (!recorder || recorder.state === 'inactive') {
      return
    }
    recorder.stop()
  }

  function toggleRecording() {
    if (recording) {
      stopRecording()
    } else {
      startRecording()
    }
  }

  function addWaypointFromCanvas(event: React.MouseEvent<HTMLCanvasElement>) {
    if (!mapGeometry || !canvasRef.current) {
      appendLog('error', '地图还未就绪，暂时不能标点')
      return
    }
    const rect = canvasRef.current.getBoundingClientRect()
    const transform = createMapTransform(mapGeometry, {
      width: canvasRef.current.width,
      height: canvasRef.current.height,
    })
    const point = canvasToWorld(
      {
        x: (event.clientX - rect.left) * window.devicePixelRatio,
        y: (event.clientY - rect.top) * window.devicePixelRatio,
      },
      mapGeometry,
      transform,
    )
    setWaypoints((current) => [...current, point])
  }

  function removeWaypoint(index: number) {
    setWaypoints((current) => current.filter((_waypoint, currentIndex) => currentIndex !== index))
  }

  function moveWaypoint(index: number, direction: -1 | 1) {
    setWaypoints((current) => {
      const nextIndex = index + direction
      if (nextIndex < 0 || nextIndex >= current.length) {
        return current
      }
      const copy = [...current]
      const [item] = copy.splice(index, 1)
      copy.splice(nextIndex, 0, item)
      return copy
    })
  }

  function clearWaypoints() {
    setWaypoints([])
    appendLog('info', '已清空预览航点')
  }

  function sendNavigation() {
    if (!waypoints.length) {
      appendLog('error', '请先在地图上标点')
      return
    }
    if (navigationReady !== true) {
      appendLog(
        'error',
        navigationReady === false ? '导航尚未就绪，不能下发航点' : '尚未收到导航就绪信号',
      )
      return
    }
    if (!navClientRef.current || navConnectionStatus !== 'connected') {
      appendLog('error', '导航桥未连接，不能下发航点')
      return
    }
    try {
      navClientRef.current.send(createNavigateCommand(waypoints))
      appendLog('info', `已提交 ${waypoints.length} 个航点`)
    } catch (error) {
      appendLog('error', error instanceof Error ? error.message : '导航命令发送失败')
    }
  }

  function cancelNavigation() {
    if (!navClientRef.current || navConnectionStatus !== 'connected') {
      appendLog('error', '导航桥未连接，不能取消导航')
      return
    }
    try {
      navClientRef.current.send({ type: 'cancel_navigation' })
      appendLog('info', '已请求取消导航')
    } catch (error) {
      appendLog('error', error instanceof Error ? error.message : '取消导航失败')
    }
  }

  function reconnectAll() {
    setAuxiliaryStreamsEnabled(true)
    setAuxiliaryReconnectSignal((current) => current + 1)
    startCameraConnection(cameraUrl, 'manual')
    startNavConnection(navUrl, 'manual')
  }

  const drawMapCanvas = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) {
      return
    }
    const rect = canvas.getBoundingClientRect()
    const ratio = window.devicePixelRatio || 1
    const width = Math.max(1, Math.round(rect.width * ratio))
    const height = Math.max(1, Math.round(rect.height * ratio))
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width
      canvas.height = height
    }

    const context = canvas.getContext('2d')
    if (!context) {
      return
    }
    context.clearRect(0, 0, width, height)
    drawMapBackground(context, width, height)

    if (!mapData || !mapGeometry) {
      drawEmptyMapState(context, width, height)
      return
    }

    const transform = createMapTransform(mapGeometry, { width, height })
    drawOccupancyGrid(context, mapData, mapGeometry, transform)
    drawCostmap(context, costmaps.global, mapGeometry, transform, 'rgba(255, 97, 97, 0.74)')
    drawCostmap(context, costmaps.local, mapGeometry, transform, 'rgba(255, 197, 51, 0.78)')
    drawPath(context, paths.global, mapGeometry, transform, '#57c1ff', 3 * ratio)
    drawPath(context, paths.local, mapGeometry, transform, '#59d499', 2 * ratio)
    drawWaypoints(context, waypoints, mapGeometry, transform, ratio)
    drawRobot(context, robotPose, mapGeometry, transform, ratio)
  }, [costmaps, mapData, mapGeometry, paths, robotPose, waypoints])

  useEffect(() => {
    drawMapCanvas()
    const onResize = () => drawMapCanvas()
    window.addEventListener('resize', onResize)
    return () => window.removeEventListener('resize', onResize)
  }, [drawMapCanvas])

  const globalObstacleCount = costmaps.global?.cells.length ?? 0
  const localObstacleCount = costmaps.local?.cells.length ?? 0
  const navigationReadiness: NavigationReadinessState =
    navigationReady === true ? 'available' : navigationReady === false ? 'unavailable' : 'unknown'

  async function launchOperator() {
    const launched = await taskControl.runFlow('operator')
    if (!launched) {
      setTaskDrawerOpen(true)
    }
  }

  return (
    <main className="app-shell">
      <aside className="left-rail" aria-label="系统状态">
        <div className="brand">
          <div className="brand-mark" aria-hidden="true">
            <Navigation size={19} />
          </div>
          <div>
            <h1>SLAM Nav 导航作业台</h1>
            <p>RGB 视野 · Nav2 地图 · 航点预览</p>
          </div>
        </div>

        <div className="brand-actions" aria-label="工作区入口">
          <button
            className="tool-button"
            type="button"
            onClick={() => setTaskDrawerOpen(true)}
            aria-expanded={taskDrawerOpen}
            aria-controls="task-control-drawer"
            title="打开任务控制"
          >
            <ListTodo size={15} />
            任务控制
            {taskControl.state?.active.length ? (
              <span className="action-count" aria-label={`${taskControl.state.active.length} 个运行任务`}>
                {taskControl.state.active.length}
              </span>
            ) : null}
          </button>
          <button
            className="tool-button professional-button"
            type="button"
            onClick={() => void launchOperator()}
            disabled={
              !operatorAvailable ||
              operatorRunning ||
              taskControl.actionKey !== null
            }
            title={operatorButtonTitle}
            aria-label={operatorRunning ? '专业界面已运行' : '打开专业界面'}
          >
            <MonitorUp size={15} />
            专业界面
            <span className={`operator-status-dot ${operatorRunning ? 'running' : ''}`} aria-hidden="true" />
          </button>
        </div>

        <section className="rail-section">
          <h2>链路</h2>
          <StatusRow icon={<Camera size={15} />} label="相机桥" status={cameraStatus} />
          <StatusRow icon={<Map size={15} />} label="导航桥" status={navConnectionStatus} />
          <StatusRow icon={<Route size={15} />} label="Nav2" status={navStatus.state} />
          <StatusRow icon={<CircleDot size={15} />} label="导航许可" status={navigationReadiness} />
        </section>

        <section className="rail-section">
          <h2>话题</h2>
          <TopicRow label="相机" value={frame?.topic ?? DEFAULT_CAMERA_TOPIC} />
          <TopicRow label="地图" value={mapData?.topic ?? DEFAULT_MAP_TOPIC} />
          <TopicRow label="全局障碍" value={String(globalObstacleCount)} />
          <TopicRow label="局部障碍" value={String(localObstacleCount)} />
        </section>

        <section className="rail-section rail-actions">
          <button className="tool-button" type="button" onClick={reconnectAll}>
            <RotateCcw size={15} />
            重连
          </button>
          <button className="tool-button" type="button" onClick={disconnectAll}>
            <WifiOff size={15} />
            断开
          </button>
        </section>

        <section className="rail-section rail-log">
          <h2>导航事件</h2>
          <div className="log-panel" aria-label="导航事件日志">
            {logs.map((entry) => (
              <div className={`log-row ${entry.level === 'error' ? 'log-error' : ''}`} key={entry.id}>
                <time>{entry.time}</time>
                <span title={entry.message}>{entry.message}</span>
              </div>
            ))}
          </div>
        </section>
      </aside>

      <section className="map-panel" aria-label="导航地图">
        <div className="panel-header">
          <div className="panel-title">
            <Map size={16} />
            <div>
              <h2>导航地图</h2>
              <span>点击添加航点，确认后下发</span>
            </div>
          </div>
          <div className="map-legend" aria-label="地图图例">
            <LegendDot color="#ff6161" label="全局障碍" />
            <LegendDot color="#ffc533" label="局部障碍" />
            <LegendDot color="#57c1ff" label="全局路径" />
            <LegendDot color="#59d499" label="局部路径" />
          </div>
        </div>

        <div className="map-viewport">
          <canvas
            ref={canvasRef}
            className="map-canvas"
            aria-label="可标点导航地图"
            onClick={addWaypointFromCanvas}
          />
          <div className="map-hud" aria-label="导航状态">
            <span className="hud-chip">
              <CircleDot size={12} />
              {NAV_STATE_LABELS[navStatus.state]}
            </span>
            <span className={`hud-chip navigation-readiness ${navigationReadiness}`}>
              <CircleDot size={12} />
              {navigationReady === true ? '导航已就绪' : navigationReady === false ? '导航未就绪' : '检测导航就绪'}
            </span>
            <span className="hud-chip">航点 {waypoints.length}</span>
            <span className="hud-chip">地图 {mapData ? `${mapData.width} x ${mapData.height}` : '--'}</span>
            <span className="hud-chip">机器人 {robotPose ? formatPose(robotPose) : '--'}</span>
          </div>
        </div>
      </section>

      <aside className="right-stack" aria-label="相机与航点控制">
        <section className="vision-panel" aria-label="机器人视觉">
          <div className="panel-header compact">
            <div className="panel-title">
              <Radio size={16} />
              <div>
                <h2>机器人视觉</h2>
                <span>导航与 Piper RGB-D 四路视野</span>
              </div>
            </div>
          </div>

          <div className="vision-grid">
            <article className="vision-tile" aria-label="导航 RGB">
              <header className="vision-tile-header">
                <div>
                  <Camera size={14} />
                  <strong>导航 RGB</strong>
                </div>
                <div className="vision-tile-actions">
                  <button
                    type="button"
                    onClick={captureSnapshot}
                    disabled={!frame}
                    aria-label="截图"
                    title="截图导航 RGB"
                  >
                    <Save size={13} />
                  </button>
                  <button type="button" onClick={requestCameraFullscreen} aria-label="全屏相机" title="全屏导航 RGB">
                    <Expand size={13} />
                  </button>
                </div>
              </header>
              <div className="vision-viewport" id="nav-rgb-view">
                {frame ? (
                  <img
                    ref={imageRef}
                    className="video-frame"
                    src={frame.imageUrl}
                    alt="导航相机 RGB 实时画面"
                  />
                ) : (
                  <div className="empty-state compact-empty-state">
                    <Activity size={20} />
                    <strong>{paused ? '画面已暂停' : `等待 ${DEFAULT_CAMERA_TOPIC}`}</strong>
                  </div>
                )}
                <span className={`stream-status ${cameraStatus}`}>{STATUS_LABELS[cameraStatus]}</span>
                <div className="nav-stream-controls">
                  <button type="button" onClick={togglePaused} aria-label={paused ? '恢复' : '暂停'} title={paused ? '恢复' : '暂停'}>
                    {paused ? <Play size={13} /> : <Pause size={13} />}
                  </button>
                  <button
                    type="button"
                    className={recording ? 'recording' : ''}
                    onClick={toggleRecording}
                    disabled={!recording && !frame}
                    aria-label={recording ? '停止录制' : '开始录制'}
                    title={recording ? '停止录制' : '开始录制'}
                  >
                    {recording ? <Square size={13} /> : <Video size={13} />}
                  </button>
                </div>
              </div>
              <footer className="vision-meta">
                <span>{formatNumber(frame?.fps)} FPS</span>
                <span>{formatLatency(frame?.latencyMs)}</span>
                <span>{formatResolution(frame)}</span>
              </footer>
            </article>

            <CameraStreamTile
              label="导航深度"
              topic={DEFAULT_NAV_DEPTH_TOPIC}
              wsPath="/ws/nav/depth"
              configuredUrl={import.meta.env.VITE_NAV_DEPTH_WS_URL}
              enabled={auxiliaryStreamsEnabled}
              reconnectSignal={auxiliaryReconnectSignal}
              depth
              minDepthM={0.25}
              maxDepthM={5}
            />
            <CameraStreamTile
              label="Piper RGB"
              topic="/piper/arm_camera/color/image_raw"
              wsPath="/ws/piper/rgb"
              configuredUrl={import.meta.env.VITE_PIPER_RGB_WS_URL}
              enabled={auxiliaryStreamsEnabled}
              reconnectSignal={auxiliaryReconnectSignal}
            />
            <CameraStreamTile
              label="Piper 深度"
              topic="/piper/arm_camera/depth/image_raw"
              wsPath="/ws/piper/depth"
              configuredUrl={import.meta.env.VITE_PIPER_DEPTH_WS_URL}
              enabled={auxiliaryStreamsEnabled}
              reconnectSignal={auxiliaryReconnectSignal}
              depth
              minDepthM={0.15}
              maxDepthM={2.5}
            />
          </div>
        </section>

        <section className="waypoint-panel" aria-label="航点队列">
          <div className="panel-header compact">
            <div className="panel-title">
              <Flag size={16} />
              <div>
                <h2>航点队列</h2>
                <span>
                  {!waypoints.length
                    ? '在地图上点击标点'
                    : navigationReady === true
                      ? '导航就绪，确认后下发'
                      : navigationReady === false
                        ? '导航未就绪'
                        : '等待导航就绪信号'}
                </span>
              </div>
            </div>
            <button className="icon-button" type="button" onClick={clearWaypoints} aria-label="清空航点">
              <Trash2 size={16} />
            </button>
          </div>

          <div className="waypoint-list">
            {waypoints.length ? (
              waypoints.map((waypoint, index) => (
                <div className="waypoint-row" key={`${waypoint.x}-${waypoint.y}-${index}`}>
                  <span className="waypoint-index">{index + 1}</span>
                  <span className="waypoint-coordinates">{formatWaypoint(waypoint)}</span>
                  <button
                    className="mini-button"
                    type="button"
                    onClick={() => moveWaypoint(index, -1)}
                    disabled={index === 0}
                    aria-label={`上移航点 ${index + 1}`}
                  >
                    ↑
                  </button>
                  <button
                    className="mini-button"
                    type="button"
                    onClick={() => moveWaypoint(index, 1)}
                    disabled={index === waypoints.length - 1}
                    aria-label={`下移航点 ${index + 1}`}
                  >
                    ↓
                  </button>
                  <button
                    className="mini-button danger"
                    type="button"
                    onClick={() => removeWaypoint(index)}
                    aria-label={`删除航点 ${index + 1}`}
                  >
                    <X size={13} />
                  </button>
                </div>
              ))
            ) : (
              <div className="empty-waypoints">
                <LocateFixed size={22} />
                <span>暂无预览航点</span>
              </div>
            )}
          </div>

          <div className="nav-controls">
            <button
              className="primary-button"
              type="button"
              onClick={sendNavigation}
              disabled={
                !waypoints.length ||
                navConnectionStatus !== 'connected' ||
                navigationReady !== true
              }
              title={
                navigationReady === true
                  ? '开始导航'
                  : navigationReady === false
                    ? '导航尚未就绪'
                    : '等待 /navigation_ready'
              }
            >
              <Send size={15} />
              开始导航
            </button>
            <button
              className="tool-button"
              type="button"
              onClick={cancelNavigation}
              disabled={navConnectionStatus !== 'connected'}
            >
              <Square size={15} />
              取消导航
            </button>
          </div>
        </section>

        <section className="connection-panel" aria-label="连接设置">
          <div className="input-group">
            <label htmlFor="camera-url">相机桥</label>
            <input id="camera-url" value={cameraUrl} onChange={(event) => setCameraUrl(event.target.value)} />
          </div>
          <div className="input-group">
            <label htmlFor="nav-url">导航桥</label>
            <input id="nav-url" value={navUrl} onChange={(event) => setNavUrl(event.target.value)} />
          </div>
        </section>
      </aside>

      {taskDrawerOpen ? (
        <TaskDrawer
          open
          state={taskControl.state}
          loading={taskControl.loading}
          error={taskControl.error}
          actionKey={taskControl.actionKey}
          onClose={() => setTaskDrawerOpen(false)}
          onRefresh={taskControl.refresh}
          onRun={taskControl.runFlow}
          onStop={taskControl.stopFlow}
        />
      ) : null}
    </main>
  )
}

function StatusRow({
  icon,
  label,
  status,
}: {
  icon: React.ReactNode
  label: string
  status: ConnectionStatus | NavStatusMessage['state'] | NavigationReadinessState
}) {
  const statusText =
    status in STATUS_LABELS
      ? STATUS_LABELS[status as ConnectionStatus]
      : status in NAVIGATION_READINESS_LABELS
        ? NAVIGATION_READINESS_LABELS[status as NavigationReadinessState]
        : NAV_STATE_LABELS[status as NavStatusMessage['state']]
  return (
    <div className={`status-row ${status}`}>
      {icon}
      <span>{label}</span>
      <strong title={statusText}>{statusText}</strong>
    </div>
  )
}

function TopicRow({ label, value }: { label: string; value: string }) {
  return (
    <div className="topic-row">
      <span>{label}</span>
      <strong title={value}>{value}</strong>
    </div>
  )
}

function LegendDot({ color, label }: { color: string; label: string }) {
  return (
    <span className="legend-item">
      <span style={{ background: color }} />
      {label}
    </span>
  )
}

function drawMapBackground(context: CanvasRenderingContext2D, width: number, height: number) {
  context.fillStyle = '#08090b'
  context.fillRect(0, 0, width, height)
  context.strokeStyle = 'rgba(255,255,255,0.035)'
  context.lineWidth = 1
  const step = 36 * (window.devicePixelRatio || 1)
  for (let x = 0; x < width; x += step) {
    context.beginPath()
    context.moveTo(x, 0)
    context.lineTo(x, height)
    context.stroke()
  }
  for (let y = 0; y < height; y += step) {
    context.beginPath()
    context.moveTo(0, y)
    context.lineTo(width, y)
    context.stroke()
  }
}

function drawEmptyMapState(context: CanvasRenderingContext2D, width: number, height: number) {
  context.fillStyle = '#f4f4f6'
  context.font = `${16 * (window.devicePixelRatio || 1)}px Inter, sans-serif`
  context.textAlign = 'center'
  context.fillText('等待 /map 地图数据', width / 2, height / 2)
}

function drawOccupancyGrid(
  context: CanvasRenderingContext2D,
  mapData: MapMessage,
  geometry: OccupancyGridGeometry,
  transform: ReturnType<typeof createMapTransform>,
) {
  const imageData = document.createElement('canvas')
  imageData.width = mapData.width
  imageData.height = mapData.height
  const imageContext = imageData.getContext('2d')
  if (!imageContext) {
    return
  }
  const pixels = imageContext.createImageData(mapData.width, mapData.height)
  for (let index = 0; index < mapData.data.length; index += 1) {
    const value = mapData.data[index]
    const row = Math.floor(index / mapData.width)
    const column = index % mapData.width
    const target = ((mapData.height - row - 1) * mapData.width + column) * 4
    const occupied = value >= 65
    const unknown = value < 0
    const shade = occupied ? 214 : unknown ? 38 : 16
    pixels.data[target] = shade
    pixels.data[target + 1] = shade
    pixels.data[target + 2] = occupied ? 216 : shade
    pixels.data[target + 3] = unknown ? 115 : occupied ? 235 : 185
  }
  imageContext.putImageData(pixels, 0, 0)
  context.imageSmoothingEnabled = false
  context.drawImage(
    imageData,
    transform.offsetX,
    transform.offsetY,
    geometry.width * geometry.resolution * transform.scale,
    geometry.height * geometry.resolution * transform.scale,
  )
}

function drawCostmap(
  context: CanvasRenderingContext2D,
  costmap: CostmapMessage | null,
  fallbackGeometry: OccupancyGridGeometry,
  fallbackTransform: ReturnType<typeof createMapTransform>,
  color: string,
) {
  if (!costmap) {
    return
  }
  const geometry = {
    width: costmap.width,
    height: costmap.height,
    resolution: costmap.resolution,
    origin: costmap.origin,
  }
  context.fillStyle = color
  if (costmap.points.length) {
    for (const point of costmap.points) {
      const canvasPoint = worldToCanvas(point, fallbackGeometry, fallbackTransform)
      const size = Math.max(1.5, costmap.resolution * fallbackTransform.scale)
      context.fillRect(canvasPoint.x - size / 2, canvasPoint.y - size / 2, size, size)
    }
    return
  }
  if (costmap.frame && costmap.frame !== 'map') {
    return
  }
  for (const cell of costmap.cells) {
    const world = gridCellToWorld(cell, geometry)
    const point = worldToCanvas(world, fallbackGeometry, fallbackTransform)
    const size = Math.max(1.5, costmap.resolution * fallbackTransform.scale)
    context.fillRect(point.x - size / 2, point.y - size / 2, size, size)
  }
}

function drawPath(
  context: CanvasRenderingContext2D,
  path: PathMessage | null,
  geometry: OccupancyGridGeometry,
  transform: ReturnType<typeof createMapTransform>,
  color: string,
  lineWidth: number,
) {
  if (!path || path.points.length < 2) {
    return
  }
  context.strokeStyle = color
  context.lineWidth = lineWidth
  context.lineJoin = 'round'
  context.lineCap = 'round'
  context.beginPath()
  path.points.forEach((point, index) => {
    const canvasPoint = worldToCanvas(point, geometry, transform)
    if (index === 0) {
      context.moveTo(canvasPoint.x, canvasPoint.y)
    } else {
      context.lineTo(canvasPoint.x, canvasPoint.y)
    }
  })
  context.stroke()
}

function drawWaypoints(
  context: CanvasRenderingContext2D,
  waypoints: WaypointDraft[],
  geometry: OccupancyGridGeometry,
  transform: ReturnType<typeof createMapTransform>,
  ratio: number,
) {
  if (!waypoints.length) {
    return
  }
  context.strokeStyle = '#ffffff'
  context.lineWidth = 1.5 * ratio
  context.beginPath()
  waypoints.forEach((waypoint, index) => {
    const point = worldToCanvas(waypoint, geometry, transform)
    if (index === 0) {
      context.moveTo(point.x, point.y)
    } else {
      context.lineTo(point.x, point.y)
    }
  })
  context.stroke()

  waypoints.forEach((waypoint, index) => {
    const point = worldToCanvas(waypoint, geometry, transform)
    context.fillStyle = '#ffffff'
    context.beginPath()
    context.arc(point.x, point.y, 9 * ratio, 0, Math.PI * 2)
    context.fill()
    context.fillStyle = '#000000'
    context.font = `600 ${11 * ratio}px Inter, sans-serif`
    context.textAlign = 'center'
    context.textBaseline = 'middle'
    context.fillText(String(index + 1), point.x, point.y)
  })
}

function drawRobot(
  context: CanvasRenderingContext2D,
  robotPose: RobotPoseMessage | null,
  geometry: OccupancyGridGeometry,
  transform: ReturnType<typeof createMapTransform>,
  ratio: number,
) {
  if (!robotPose) {
    return
  }
  const point = worldToCanvas(robotPose, geometry, transform)
  context.save()
  context.translate(point.x, point.y)
  context.rotate(-robotPose.yaw)
  context.fillStyle = '#59d499'
  context.strokeStyle = '#000000'
  context.lineWidth = 1.5 * ratio
  context.beginPath()
  context.moveTo(14 * ratio, 0)
  context.lineTo(-10 * ratio, -8 * ratio)
  context.lineTo(-6 * ratio, 0)
  context.lineTo(-10 * ratio, 8 * ratio)
  context.closePath()
  context.fill()
  context.stroke()
  context.restore()
}

function createLog(level: LogEntry['level'], message: string): LogEntry {
  return {
    id: Date.now() + Math.random(),
    level,
    message,
    time: new Date().toLocaleTimeString(),
  }
}

function formatNumber(value?: number) {
  return typeof value === 'number' ? value.toFixed(1) : '--'
}

function formatLatency(value?: number) {
  return typeof value === 'number' ? `${Math.round(value)} ms` : '--'
}

function formatResolution(frame: CameraFrame | null) {
  if (!frame?.width || !frame.height) {
    return '--'
  }

  return `${frame.width}x${frame.height}`
}

function selectRecordingMimeType() {
  if (typeof MediaRecorder.isTypeSupported !== 'function') {
    return ''
  }
  const candidates = [
    'video/webm;codecs=vp9',
    'video/webm;codecs=vp8',
    'video/webm',
    'video/mp4',
  ]
  return candidates.find((candidate) => MediaRecorder.isTypeSupported(candidate)) ?? ''
}

function formatFileSize(bytes: number) {
  if (bytes < 1024 * 1024) {
    return `${Math.max(1, Math.round(bytes / 1024))} KB`
  }
  return `${(bytes / 1024 / 1024).toFixed(1)} MB`
}

function formatPose(pose: RobotPoseMessage) {
  return `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}`
}

function formatWaypoint(waypoint: WaypointDraft) {
  return `${waypoint.x.toFixed(2)}, ${waypoint.y.toFixed(2)}`
}

export default App
