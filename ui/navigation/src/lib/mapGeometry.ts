export type OccupancyGridGeometry = {
  width: number
  height: number
  resolution: number
  origin: {
    x: number
    y: number
    yaw: number
  }
}

export type CanvasSize = {
  width: number
  height: number
}

export type MapTransform = {
  scale: number
  offsetX: number
  offsetY: number
}

export type Point2D = {
  x: number
  y: number
}

export function createMapTransform(
  geometry: OccupancyGridGeometry,
  canvas: CanvasSize,
  padding = 0,
): MapTransform {
  const mapWidthMeters = geometry.width * geometry.resolution
  const mapHeightMeters = geometry.height * geometry.resolution
  const availableWidth = Math.max(1, canvas.width - padding * 2)
  const availableHeight = Math.max(1, canvas.height - padding * 2)
  const scale = Math.min(availableWidth / mapWidthMeters, availableHeight / mapHeightMeters)
  const renderedWidth = mapWidthMeters * scale
  const renderedHeight = mapHeightMeters * scale

  return {
    scale,
    offsetX: (canvas.width - renderedWidth) / 2,
    offsetY: (canvas.height - renderedHeight) / 2,
  }
}

export function worldToCanvas(
  point: Point2D,
  geometry: OccupancyGridGeometry,
  transform: MapTransform,
): Point2D {
  const localX = (point.x - geometry.origin.x) / geometry.resolution
  const localY = (point.y - geometry.origin.y) / geometry.resolution

  return {
    x: transform.offsetX + localX * geometry.resolution * transform.scale,
    y: transform.offsetY + (geometry.height - localY) * geometry.resolution * transform.scale,
  }
}

export function canvasToWorld(
  point: Point2D,
  geometry: OccupancyGridGeometry,
  transform: MapTransform,
): Point2D {
  const metersX = (point.x - transform.offsetX) / transform.scale
  const metersYFromTop = (point.y - transform.offsetY) / transform.scale
  const metersY = geometry.height * geometry.resolution - metersYFromTop

  return {
    x: geometry.origin.x + metersX,
    y: geometry.origin.y + metersY,
  }
}

export function gridCellToWorld(
  cell: Point2D,
  geometry: OccupancyGridGeometry,
): Point2D {
  return {
    x: geometry.origin.x + (cell.x + 0.5) * geometry.resolution,
    y: geometry.origin.y + (cell.y + 0.5) * geometry.resolution,
  }
}

