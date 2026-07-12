import { useCallback, useEffect, useRef, useState } from 'react'
import {
  fetchTaskState,
  runTaskFlow,
  stopTaskFlow,
  type TaskControlState,
} from './taskControl'

const TASK_STATE_POLL_MS = 2_500

export function useTaskControl() {
  const [state, setState] = useState<TaskControlState | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [actionKey, setActionKey] = useState<string | null>(null)
  const mountedRef = useRef(false)
  const requestRef = useRef<Promise<void> | null>(null)
  const controllerRef = useRef<AbortController | null>(null)

  const refresh = useCallback((): Promise<void> => {
    if (requestRef.current) {
      return requestRef.current
    }
    const controller = new AbortController()
    controllerRef.current = controller
    const request = fetchTaskState(controller.signal)
      .then((nextState) => {
        if (mountedRef.current) {
          setState(nextState)
          setError(null)
        }
      })
      .catch((requestError: unknown) => {
        if (mountedRef.current && !controller.signal.aborted) {
          setError(requestError instanceof Error ? requestError.message : '任务控制服务不可用')
        }
      })
      .finally(() => {
        if (controllerRef.current === controller) {
          controllerRef.current = null
        }
        requestRef.current = null
        if (mountedRef.current) {
          setLoading(false)
        }
      })
    requestRef.current = request
    return request
  }, [])

  useEffect(() => {
    mountedRef.current = true
    let stopped = false
    let timer: number | null = null
    const poll = async () => {
      await refresh()
      if (!stopped) {
        timer = window.setTimeout(() => void poll(), TASK_STATE_POLL_MS)
      }
    }
    void poll()
    return () => {
      stopped = true
      mountedRef.current = false
      if (timer !== null) {
        window.clearTimeout(timer)
      }
      controllerRef.current?.abort()
      controllerRef.current = null
    }
  }, [refresh])

  const runFlow = useCallback(async (flowId: string, args: string[] = []) => {
    setActionKey(`run:${flowId}`)
    setError(null)
    try {
      await runTaskFlow(flowId, args)
      if (mountedRef.current) {
        controllerRef.current?.abort()
        await requestRef.current
        await refresh()
      }
      return true
    } catch (requestError) {
      if (mountedRef.current) {
        setError(requestError instanceof Error ? requestError.message : '流程启动失败')
      }
      return false
    } finally {
      if (mountedRef.current) {
        setActionKey(null)
      }
    }
  }, [refresh])

  const stopFlow = useCallback(async (runId?: string) => {
    setActionKey(runId ? `stop:${runId}` : 'stop:all')
    setError(null)
    try {
      await stopTaskFlow(runId)
      if (mountedRef.current) {
        controllerRef.current?.abort()
        await requestRef.current
        await refresh()
      }
      return true
    } catch (requestError) {
      if (mountedRef.current) {
        setError(requestError instanceof Error ? requestError.message : '流程停止失败')
      }
      return false
    } finally {
      if (mountedRef.current) {
        setActionKey(null)
      }
    }
  }, [refresh])

  return { state, loading, error, actionKey, refresh, runFlow, stopFlow }
}
