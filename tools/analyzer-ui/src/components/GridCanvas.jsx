import { useRef, useEffect, useMemo } from 'react'
import { getIndexForTime } from '../utils/csvParser'

// Wall bitmask constants (mirror Types.h)
const TOP = 0b1000
const RIGHT = 0b0100
const DOWN = 0b0010
const LEFT = 0b0001

const GRID_N = 16    // constexpr int N = 16
const CELL_M = 0.2   // CELL_SIZE_METERS from Constants.h — each cell = 0.2 m

// ── Shared draw helpers ─────────────────────────────────────────────────────

function drawGrid(ctx, W, H) {
    const cellW = W / GRID_N
    const cellH = H / GRID_N

    // Dark cell fills (alternating subtle checker for readability)
    for (let i = 0; i < GRID_N; i++) {
        for (let j = 0; j < GRID_N; j++) {
            ctx.fillStyle = (i + j) % 2 === 0
                ? 'rgba(255,255,255,0.015)'
                : 'rgba(0,0,0,0.0)'
            ctx.fillRect(i * cellW, j * cellH, cellW, cellH)
        }
    }

    // Inner grid lines — crisp, 1 px
    ctx.strokeStyle = 'rgba(255,255,255,0.18)'
    ctx.lineWidth = 1
    for (let i = 1; i < GRID_N; i++) {
        // vertical
        ctx.beginPath()
        ctx.moveTo(i * cellW, 0)
        ctx.lineTo(i * cellW, H)
        ctx.stroke()
        // horizontal
        ctx.beginPath()
        ctx.moveTo(0, i * cellH)
        ctx.lineTo(W, i * cellH)
        ctx.stroke()
    }

    // Outer border — bright, 2 px
    ctx.strokeStyle = 'rgba(255,255,255,0.55)'
    ctx.lineWidth = 2
    ctx.strokeRect(1, 1, W - 2, H - 2)

    return { cellW, cellH }
}

function drawRobot(ctx, W, H, cellW, cellH, xs, ys, thetas, idx) {
    // World → canvas  (origin bottom-left, y-up in world, y-down on canvas)
    const toScreen = (wx, wy) => ({
        sx: (wx / CELL_M) * cellW,
        sy: H - (wy / CELL_M) * cellH,
    })

    if (!xs.length) return

    // Full path — very dim
    ctx.beginPath()
    for (let i = 0; i < xs.length; i++) {
        const { sx, sy } = toScreen(xs[i], ys[i])
        i === 0 ? ctx.moveTo(sx, sy) : ctx.lineTo(sx, sy)
    }
    ctx.strokeStyle = 'rgba(0,160,255,0.18)'
    ctx.lineWidth = 1.5
    ctx.lineJoin = 'round'
    ctx.stroke()

    // Past trail — bright
    ctx.beginPath()
    for (let i = 0; i <= idx; i++) {
        const { sx, sy } = toScreen(xs[i], ys[i])
        i === 0 ? ctx.moveTo(sx, sy) : ctx.lineTo(sx, sy)
    }
    ctx.strokeStyle = 'rgba(0,210,255,0.90)'
    ctx.lineWidth = 2
    ctx.stroke()

    // Current dot
    const cur = toScreen(xs[idx], ys[idx])
    const r = Math.min(cellW, cellH) * 0.24

    ctx.beginPath()
    ctx.arc(cur.sx, cur.sy, r, 0, Math.PI * 2)
    ctx.fillStyle = '#00ff88'
    ctx.fill()
    ctx.strokeStyle = 'rgba(255,255,255,0.9)'
    ctx.lineWidth = 1.8
    ctx.stroke()

    // Heading arrow
    const theta = thetas[idx] ?? 0
    const arrowLen = r * 2.4
    const ax = cur.sx + Math.cos(theta) * arrowLen
    const ay = cur.sy - Math.sin(theta) * arrowLen

    ctx.beginPath()
    ctx.moveTo(cur.sx, cur.sy)
    ctx.lineTo(ax, ay)
    ctx.strokeStyle = '#ff4444'
    ctx.lineWidth = 2.5
    ctx.lineCap = 'round'
    ctx.stroke()

    // Arrowhead
    const ang = Math.atan2(-(ay - cur.sy), ax - cur.sx)
    const hw = 5
    ctx.beginPath()
    ctx.moveTo(ax, ay)
    ctx.lineTo(ax - hw * Math.cos(ang - 0.4), ay + hw * Math.sin(ang - 0.4))
    ctx.lineTo(ax - hw * Math.cos(ang + 0.4), ay + hw * Math.sin(ang + 0.4))
    ctx.closePath()
    ctx.fillStyle = '#ff4444'
    ctx.fill()
}

// ── Component ───────────────────────────────────────────────────────────────

export default function GridCanvas({ csvData, colX, colY, colTheta, currentTimeUs }) {
    const canvasRef = useRef(null)

    const { xs, ys, thetas } = useMemo(() => {
        if (!csvData || !colX || !colY) return { xs: [], ys: [], thetas: [] }
        return {
            xs: csvData.columns[colX] ?? [],
            ys: csvData.columns[colY] ?? [],
            thetas: colTheta ? (csvData.columns[colTheta] ?? []) : [],
        }
    }, [csvData, colX, colY, colTheta])

    const hasMazeData = useMemo(() =>
        !!csvData?.headers.some((h) => h.startsWith('walls[')),
        [csvData])

    const mazeAccessors = useMemo(() => {
        if (!hasMazeData || !csvData) return null
        return {
            walls: (i, j) => csvData.columns[`walls[${i}][${j}]`],
            explored: (i, j) => csvData.columns[`explored[${i}][${j}]`],
        }
    }, [hasMazeData, csvData])

    // ── Draw ─────────────────────────────────────────────────────────────────
    useEffect(() => {
        const canvas = canvasRef.current
        if (!canvas) return
        const ctx = canvas.getContext('2d')
        const W = canvas.width
        const H = canvas.height
        if (W === 0 || H === 0) return

        // Background
        ctx.fillStyle = '#0d1117'
        ctx.fillRect(0, 0, W, H)

        const idx = getIndexForTime(csvData?.timestamps ?? [], currentTimeUs)
        const { cellW, cellH } = drawGrid(ctx, W, H)

        if (hasMazeData && mazeAccessors) {
            // ── Maze mode: explored fills + wall segments ─────────────────
            const wallPx = Math.max(2, cellW * 0.06)

            for (let i = 0; i < GRID_N; i++) {
                for (let j = 0; j < GRID_N; j++) {
                    const screenRow = GRID_N - 1 - j
                    const px = i * cellW
                    const py = screenRow * cellH

                    // Explored highlight
                    const expArr = mazeAccessors.explored(i, j)
                    if (expArr?.[idx] > 0) {
                        ctx.fillStyle = 'rgba(88,166,255,0.13)'
                        ctx.fillRect(px + 1, py + 1, cellW - 2, cellH - 2)
                    }

                    // Wall segments
                    const wallArr = mazeAccessors.walls(i, j)
                    const bits = wallArr?.[idx] ?? 0
                    if (!bits) continue

                    ctx.strokeStyle = '#d0e8ff'
                    ctx.lineWidth = wallPx
                    ctx.lineCap = 'square'

                    if (bits & TOP) { ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px + cellW, py); ctx.stroke() }
                    if (bits & DOWN) { ctx.beginPath(); ctx.moveTo(px, py + cellH); ctx.lineTo(px + cellW, py + cellH); ctx.stroke() }
                    if (bits & LEFT) { ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px, py + cellH); ctx.stroke() }
                    if (bits & RIGHT) { ctx.beginPath(); ctx.moveTo(px + cellW, py); ctx.lineTo(px + cellW, py + cellH); ctx.stroke() }
                }
            }
        }

        // Robot always drawn on top
        drawRobot(ctx, W, H, cellW, cellH, xs, ys, thetas, idx)

    }, [xs, ys, thetas, currentTimeUs, hasMazeData, mazeAccessors, csvData])

    // Resize observer
    useEffect(() => {
        const canvas = canvasRef.current
        if (!canvas) return
        const ro = new ResizeObserver(() => {
            canvas.width = canvas.clientWidth
            canvas.height = canvas.clientHeight
        })
        ro.observe(canvas)
        return () => ro.disconnect()
    }, [])

    return (
        <div className="panel grid-panel">
            <h2 className="panel-title">
                <span className="icon">🗺</span> 2D Grid
                {hasMazeData && <span className="badge-maze">maze</span>}
            </h2>
            {(!colX || !colY) ? (
                <p className="empty-msg">Select X and Y columns in Controls</p>
            ) : (
                <canvas ref={canvasRef} className="grid-canvas" />
            )}
        </div>
    )
}
