export type TaskFlowLevel = 'primary' | 'utility' | 'check' | 'danger' | string

export type TaskFlow = {
  id: string
  label: string
  summary: string
  level: TaskFlowLevel
  command: string
}

export type TaskFlowGroup = {
  name: string
  items: TaskFlow[]
}

export type TaskRun = {
  runId: string | null
  flowId: string | null
  label: string
  command: string | string[]
  startedAt: number | null
  finishedAt: number | null
  returnCode: number | null
  stopped?: boolean
  logPath: string | null
}

export type OperatorState = {
  available: boolean
  running: boolean
  reason: string | null
}

export type TaskControlState = {
  running: boolean
  current: TaskRun
  active: TaskRun[]
  history: TaskRun[]
  flows: TaskFlowGroup[]
  operator: OperatorState
  now: number
}

type ApiErrorPayload = {
  error?: string
  message?: string
}

export class TaskControlError extends Error {
  readonly status: number

  constructor(message: string, status: number) {
    super(message)
    this.name = 'TaskControlError'
    this.status = status
  }
}

async function requestJson<T>(path: string, options?: RequestInit): Promise<T> {
  const response = await fetch(path, {
    headers: { 'Content-Type': 'application/json' },
    ...options,
  })
  const payload = (await response.json()) as T & ApiErrorPayload
  if (!response.ok) {
    throw new TaskControlError(
      payload.error || payload.message || response.statusText || '任务控制请求失败',
      response.status,
    )
  }
  return payload
}

export function fetchTaskState(signal?: AbortSignal) {
  return requestJson<TaskControlState>('/api/state', { signal })
}

export function fetchTaskLog(runId: string | null, signal?: AbortSignal) {
  const params = new URLSearchParams({ lines: '180' })
  if (runId) {
    params.set('runId', runId)
  }
  return requestJson<{ text: string }>(`/api/log?${params.toString()}`, { signal })
}

export function runTaskFlow(flowId: string, args: string[] = []) {
  return requestJson<{ ok: boolean; current?: TaskRun }>('/api/run', {
    method: 'POST',
    body: JSON.stringify({ flowId, args }),
  })
}

export function stopTaskFlow(runId?: string) {
  return requestJson<{ ok: boolean; stopped?: number; message?: string }>('/api/stop', {
    method: 'POST',
    body: JSON.stringify(runId ? { runId } : {}),
  })
}
