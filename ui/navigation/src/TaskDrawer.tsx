import {
  ChevronRight,
  CircleAlert,
  Clock3,
  History,
  ListTodo,
  LoaderCircle,
  MonitorUp,
  Play,
  RefreshCw,
  ScrollText,
  Square,
  Terminal,
  X,
} from 'lucide-react'
import { useEffect, useMemo, useRef, useState } from 'react'
import { fetchTaskLog, type TaskControlState, type TaskFlow, type TaskRun } from './lib/taskControl'

type TaskDrawerProps = {
  open: boolean
  state: TaskControlState | null
  loading: boolean
  error: string | null
  actionKey: string | null
  onClose: () => void
  onRefresh: () => Promise<void>
  onRun: (flowId: string, args?: string[]) => Promise<boolean>
  onStop: (runId?: string) => Promise<boolean>
}

const FLOW_PRIORITY = new Map([
  ['nav', 0],
  ['nav-full', 1],
  ['auto-mapping', 2],
  ['demo-nav', 3],
  ['sim-static', 4],
  ['operator', 5],
])
const EMPTY_RUNS: TaskRun[] = []

export function TaskDrawer({
  open,
  state,
  loading,
  error,
  actionKey,
  onClose,
  onRefresh,
  onRun,
  onStop,
}: TaskDrawerProps) {
  const [selectedRunId, setSelectedRunId] = useState<string | null>(null)
  const [logText, setLogText] = useState('')
  const [logError, setLogError] = useState<string | null>(null)
  const [logLoading, setLogLoading] = useState(false)
  const closeButtonRef = useRef<HTMLButtonElement | null>(null)

  const active = state?.active ?? EMPTY_RUNS
  const history = state?.history ?? EMPTY_RUNS
  const groups = useMemo(() => {
    return [...(state?.flows ?? [])]
      .sort((left, right) => groupPriority(left.name) - groupPriority(right.name))
      .map((group) => ({
        ...group,
        items: [...group.items].sort(
          (left, right) =>
            (FLOW_PRIORITY.get(left.id) ?? 100) - (FLOW_PRIORITY.get(right.id) ?? 100),
        ),
      }))
  }, [state?.flows])
  const operatorRunning = Boolean(
    state?.operator?.running || active.some((run) => run.flowId === 'operator'),
  )
  const selectedRun = findRun(selectedRunId, state)

  useEffect(() => {
    if (!open) {
      return
    }
    closeButtonRef.current?.focus()
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        onClose()
      }
    }
    window.addEventListener('keydown', onKeyDown)
    return () => window.removeEventListener('keydown', onKeyDown)
  }, [onClose, open])

  useEffect(() => {
    if (!open) {
      return
    }
    if (selectedRunId && findRun(selectedRunId, state)) {
      return
    }
    setSelectedRunId(active[0]?.runId ?? state?.current?.runId ?? history[0]?.runId ?? null)
  }, [active, history, open, selectedRunId, state])

  useEffect(() => {
    if (!open) {
      return
    }
    let mounted = true
    let controller: AbortController | null = null
    let timer: number | null = null

    const refreshLog = async () => {
      controller = new AbortController()
      const currentController = controller
      setLogLoading(true)
      try {
        const result = await fetchTaskLog(selectedRunId, currentController.signal)
        if (mounted) {
          setLogText(result.text ?? '')
          setLogError(null)
        }
      } catch (requestError) {
        if (mounted && !currentController.signal.aborted) {
          setLogError(requestError instanceof Error ? requestError.message : '无法读取流程日志')
        }
      } finally {
        if (mounted) {
          setLogLoading(false)
          timer = window.setTimeout(() => void refreshLog(), 2_000)
        }
      }
    }

    void refreshLog()
    return () => {
      mounted = false
      if (timer !== null) {
        window.clearTimeout(timer)
      }
      controller?.abort()
    }
  }, [open, selectedRunId])

  return (
    <div className={`task-drawer-layer ${open ? 'open' : ''}`} aria-hidden={!open}>
      <button className="task-drawer-backdrop" type="button" onClick={onClose} tabIndex={-1} aria-hidden="true" />
      <aside className="task-drawer" id="task-control-drawer" role="dialog" aria-modal="true" aria-labelledby="task-drawer-title">
        <header className="task-drawer-header">
          <div className="task-drawer-title">
            <ListTodo size={19} />
            <div>
              <h2 id="task-drawer-title">任务控制</h2>
              <span>{active.length ? `${active.length} 个任务运行中` : '系统流程待命'}</span>
            </div>
          </div>
          <div className="drawer-header-actions">
            <button className="icon-button" type="button" onClick={() => void onRefresh()} disabled={loading} title="刷新任务状态" aria-label="刷新任务状态">
              <RefreshCw size={16} className={loading ? 'spin' : ''} />
            </button>
            <button ref={closeButtonRef} className="icon-button" type="button" onClick={onClose} title="关闭任务控制" aria-label="关闭任务控制">
              <X size={17} />
            </button>
          </div>
        </header>

        <div className="task-drawer-body">
          {error ? (
            <div className="task-alert" role="alert">
              <CircleAlert size={16} />
              <span>{error}</span>
            </div>
          ) : null}

          <section className="operator-strip" aria-label="专业界面状态">
            <div className="operator-strip-copy">
              <MonitorUp size={18} />
              <div>
                <strong>Qt / RViz Operator</strong>
                <span>{operatorStatusText(state, operatorRunning, loading)}</span>
              </div>
            </div>
            <button
              className="primary-button operator-launch-button"
              type="button"
              onClick={() => void onRun('operator')}
              disabled={
                loading ||
                operatorRunning ||
                !state?.operator?.available ||
                actionKey !== null
              }
              title={state?.operator?.reason ?? (operatorRunning ? '专业界面已运行' : '打开专业界面')}
              aria-label="打开专业界面"
            >
              {actionKey === 'run:operator' ? <LoaderCircle className="spin" size={15} /> : <MonitorUp size={15} />}
              {operatorRunning ? '已打开' : '打开'}
            </button>
          </section>

          <section className="drawer-section" aria-labelledby="flow-list-title">
            <div className="drawer-section-heading">
              <div>
                <Terminal size={16} />
                <h3 id="flow-list-title">可用流程</h3>
              </div>
              <span>{groups.reduce((count, group) => count + group.items.length, 0)}</span>
            </div>
            <div className="flow-groups">
              {groups.map((group) => {
                const advanced = isAdvancedGroup(group.name)
                return (
                  <details className="flow-group" key={group.name} open={!advanced}>
                    <summary>
                      <ChevronRight size={15} />
                      <span>{group.name}</span>
                      <small>{group.items.length}</small>
                    </summary>
                    <div className="flow-list">
                      {group.items.map((flow) => (
                        <FlowRow
                          key={flow.id}
                          flow={flow}
                          busy={actionKey !== null}
                          operatorAvailable={Boolean(state?.operator?.available)}
                          operatorRunning={operatorRunning}
                          onRun={onRun}
                        />
                      ))}
                    </div>
                  </details>
                )
              })}
              {!loading && !groups.length ? <div className="drawer-empty">暂无可用流程</div> : null}
            </div>
          </section>

          <section className="drawer-section" aria-labelledby="active-task-title">
            <div className="drawer-section-heading">
              <div>
                <Clock3 size={16} />
                <h3 id="active-task-title">运行中</h3>
              </div>
              {active.length ? (
                <button className="text-button danger-text" type="button" onClick={() => void onStop()} disabled={actionKey !== null}>
                  <Square size={13} />
                  全部停止
                </button>
              ) : (
                <span>0</span>
              )}
            </div>
            <div className="run-list">
              {active.map((run) => (
                <RunRow
                  key={run.runId ?? `${run.flowId}-${run.startedAt}`}
                  run={run}
                  active
                  selected={selectedRunId === run.runId}
                  busy={actionKey !== null}
                  onSelect={setSelectedRunId}
                  onStop={onStop}
                />
              ))}
              {!active.length ? <div className="drawer-empty">当前没有运行任务</div> : null}
            </div>
          </section>

          <section className="drawer-section" aria-labelledby="task-history-title">
            <div className="drawer-section-heading">
              <div>
                <History size={16} />
                <h3 id="task-history-title">最近记录</h3>
              </div>
              <span>{history.length}</span>
            </div>
            <div className="run-list history-list">
              {history.map((run) => (
                <RunRow
                  key={run.runId ?? `${run.flowId}-${run.finishedAt}`}
                  run={run}
                  selected={selectedRunId === run.runId}
                  busy={false}
                  onSelect={setSelectedRunId}
                  onStop={onStop}
                />
              ))}
              {!history.length ? <div className="drawer-empty">暂无历史记录</div> : null}
            </div>
          </section>

          <section className="drawer-section process-log-section" aria-labelledby="process-log-title">
            <div className="drawer-section-heading">
              <div>
                <ScrollText size={16} />
                <h3 id="process-log-title">流程日志</h3>
              </div>
              <span>{selectedRun?.label ?? '最近任务'}</span>
            </div>
            <pre className={`process-log ${logError ? 'log-failed' : ''}`} aria-label="流程日志输出">
              {logError || logText || (logLoading ? '正在读取日志...' : '等待任务启动...')}
            </pre>
          </section>
        </div>
      </aside>
    </div>
  )
}

function FlowRow({
  flow,
  busy,
  operatorAvailable,
  operatorRunning,
  onRun,
}: {
  flow: TaskFlow
  busy: boolean
  operatorAvailable: boolean
  operatorRunning: boolean
  onRun: (flowId: string, args?: string[]) => Promise<boolean>
}) {
  const operatorDisabled = flow.id === 'operator' && (!operatorAvailable || operatorRunning)
  return (
    <article className={`flow-row level-${flow.level}`}>
      <div className="flow-copy">
        <strong>{flow.label}</strong>
        <span>{flow.summary}</span>
        <code title={flow.command}>{flow.command}</code>
      </div>
      <button
        className="icon-button flow-run-button"
        type="button"
        onClick={() => void onRun(flow.id)}
        disabled={busy || operatorDisabled}
        title={`启动${flow.label}`}
        aria-label={`启动${flow.label}`}
      >
        <Play size={15} />
      </button>
    </article>
  )
}

function RunRow({
  run,
  active = false,
  selected,
  busy,
  onSelect,
  onStop,
}: {
  run: TaskRun
  active?: boolean
  selected: boolean
  busy: boolean
  onSelect: (runId: string | null) => void
  onStop: (runId?: string) => Promise<boolean>
}) {
  const stopped = Boolean(run.stopped)
  const status = active ? '运行中' : stopped ? '已停止' : run.returnCode === 0 ? '已完成' : `退出 ${run.returnCode ?? '--'}`
  return (
    <article className={`run-row ${selected ? 'selected' : ''}`}>
      <button className="run-select" type="button" onClick={() => onSelect(run.runId)}>
        <strong>{run.label}</strong>
        <span>{formatRunTime(active ? run.startedAt : run.finishedAt)}</span>
      </button>
      <span className={`run-status ${active || stopped || run.returnCode === 0 ? 'ok' : 'failed'}`}>{status}</span>
      {active && run.runId ? (
        <button className="icon-button mini-stop" type="button" onClick={() => void onStop(run.runId!)} disabled={busy} title={`停止${run.label}`} aria-label={`停止${run.label}`}>
          <Square size={13} />
        </button>
      ) : null}
    </article>
  )
}

function findRun(runId: string | null, state: TaskControlState | null) {
  if (!runId || !state) {
    return null
  }
  return [...state.active, ...state.history, state.current].find((run) => run.runId === runId) ?? null
}

function isAdvancedGroup(name: string) {
  return /检查|维护|机械臂|高级|诊断/.test(name)
}

function groupPriority(name: string) {
  if (/导航|建图|演示|常用|启动/.test(name)) {
    return 0
  }
  if (/机械臂/.test(name)) {
    return 1
  }
  if (/检查|诊断/.test(name)) {
    return 2
  }
  if (/维护/.test(name)) {
    return 3
  }
  return 1
}

function operatorStatusText(state: TaskControlState | null, running: boolean, loading: boolean) {
  if (running) {
    return '已在工作区桌面运行'
  }
  if (loading && !state) {
    return '正在检查运行环境'
  }
  if (!state?.operator?.available) {
    return state?.operator?.reason || '当前不可用'
  }
  return '可在工作区桌面启动'
}

function formatRunTime(timestamp: number | null) {
  if (!timestamp) {
    return '--'
  }
  return new Date(timestamp * 1_000).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
}
