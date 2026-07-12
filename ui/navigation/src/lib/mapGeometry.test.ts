import { describe, expect, test } from 'vitest'
import {
  canvasToWorld,
  createMapTransform,
  worldToCanvas,
  type OccupancyGridGeometry,
} from './mapGeometry'

const geometry: OccupancyGridGeometry = {
  width: 100,
  height: 80,
  resolution: 0.05,
  origin: { x: -2.5, y: -1, yaw: 0 },
}

describe('mapGeometry', () => {
  test('fits a map inside a canvas while keeping its aspect ratio', () => {
    const transform = createMapTransform(geometry, { width: 500, height: 300 })

    expect(transform.scale).toBe(75)
    expect(transform.offsetX).toBe(62.5)
    expect(transform.offsetY).toBe(0)
  })

  test('round trips world coordinates through canvas coordinates', () => {
    const transform = createMapTransform(geometry, { width: 500, height: 300 })
    const canvasPoint = worldToCanvas({ x: 0, y: 0 }, geometry, transform)
    const worldPoint = canvasToWorld(canvasPoint, geometry, transform)

    expect(worldPoint.x).toBeCloseTo(0, 5)
    expect(worldPoint.y).toBeCloseTo(0, 5)
  })
})

