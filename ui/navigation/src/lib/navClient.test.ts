import { describe, expect, test } from 'vitest'
import {
  createNavigateCommand,
  parseNavigationMessage,
  resolveWaypointOrientations,
  type WaypointDraft,
} from './navClient'

describe('parseNavigationMessage', () => {
  test('parses map, costmap, path, robot pose, and status messages', () => {
    expect(
      parseNavigationMessage(
        JSON.stringify({
          type: 'map',
          width: 2,
          height: 2,
          resolution: 0.05,
          origin: { x: -1, y: -2, yaw: 0 },
          data: [0, 100, -1, 0],
          topic: '/map',
        }),
      ),
    ).toMatchObject({ type: 'map', width: 2, topic: '/map' })

    expect(
      parseNavigationMessage(
        JSON.stringify({
          type: 'costmap',
          scope: 'global',
          width: 2,
          height: 2,
          resolution: 0.05,
          origin: { x: 0, y: 0, yaw: 0 },
          cells: [{ x: 1, y: 1, value: 254 }],
          points: [{ x: 0.75, y: 1.25, value: 254 }],
          frame: 'map',
          sourceFrame: 'odom',
          topic: '/global_costmap/costmap',
        }),
      ),
    ).toMatchObject({
      type: 'costmap',
      scope: 'global',
      cells: [{ x: 1, y: 1, value: 254 }],
      points: [{ x: 0.75, y: 1.25, value: 254 }],
      frame: 'map',
      sourceFrame: 'odom',
    })

    expect(
      parseNavigationMessage(
        JSON.stringify({
          type: 'path',
          scope: 'local',
          points: [{ x: 1.2, y: 3.4 }],
          frame: 'map',
          sourceFrame: 'odom',
          topic: '/local_plan',
        }),
      ),
    ).toMatchObject({
      type: 'path',
      scope: 'local',
      points: [{ x: 1.2, y: 3.4 }],
      frame: 'map',
      sourceFrame: 'odom',
    })

    expect(
      parseNavigationMessage(
        JSON.stringify({ type: 'robot_pose', x: 1, y: 2, yaw: 0.5, frame: 'map', sourceFrame: 'base_footprint' }),
      ),
    ).toMatchObject({ type: 'robot_pose', x: 1, y: 2, sourceFrame: 'base_footprint' })

    expect(
      parseNavigationMessage(
        JSON.stringify({ type: 'nav_status', state: 'executing', detail: 'moving' }),
      ),
    ).toMatchObject({ type: 'nav_status', state: 'executing' })

    expect(
      parseNavigationMessage(
        JSON.stringify({ type: 'navigation_ready', ready: true, topic: '/navigation_ready' }),
      ),
    ).toEqual({ type: 'navigation_ready', ready: true, topic: '/navigation_ready' })
  })

  test('rejects malformed navigation readiness messages', () => {
    expect(() =>
      parseNavigationMessage(JSON.stringify({ type: 'navigation_ready', ready: 'yes' })),
    ).toThrow('invalid navigation ready message')
  })

  test('rejects unknown navigation messages', () => {
    expect(() => parseNavigationMessage(JSON.stringify({ type: 'mystery' }))).toThrow(
      'unsupported navigation message',
    )
  })
})

describe('resolveWaypointOrientations', () => {
  test('points each waypoint toward the next point and preserves the final segment heading', () => {
    const waypoints: WaypointDraft[] = [
      { x: 0, y: 0 },
      { x: 1, y: 0 },
      { x: 1, y: 1 },
    ]

    expect(resolveWaypointOrientations(waypoints)).toEqual([
      { x: 0, y: 0, yaw: 0 },
      { x: 1, y: 0, yaw: Math.PI / 2 },
      { x: 1, y: 1, yaw: Math.PI / 2 },
    ])
  })
})

describe('createNavigateCommand', () => {
  test('uses NavigateToPose for one waypoint and NavigateThroughPoses for multiple waypoints', () => {
    expect(createNavigateCommand([{ x: 2, y: 3, yaw: 0.25 }])).toEqual({
      type: 'navigate',
      action: 'NavigateToPose',
      waypoints: [{ x: 2, y: 3, yaw: 0.25 }],
    })

    expect(
      createNavigateCommand([
        { x: 0, y: 0 },
        { x: 1, y: 0 },
      ]),
    ).toEqual({
      type: 'navigate',
      action: 'NavigateThroughPoses',
      waypoints: [
        { x: 0, y: 0, yaw: 0 },
        { x: 1, y: 0, yaw: 0 },
      ],
    })
  })
})
