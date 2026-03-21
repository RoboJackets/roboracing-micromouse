import { useRef, useEffect, useMemo } from 'react'
import { getIndexForTime } from '../utils/csvParser'

/**
 * GridCanvas — draws the robot path on an HTML5 canvas.
 * Mirrors Dashboard::renderGridWindow().
 *
 * - Full history trail (dim blue)
 * - Already-visited path up to current time (bright blue)
 * - Current position dot (green)
 * - Heading arrow (red)
 */
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

    // Data extents for scaling
    const { minX, maxX, minY, maxY } = useMemo(() => {
        if (xs.length === 0) return { minX: -1, maxX: 1, minY: -1, maxY: 1 }
        const minX = Math.min(...xs)
        const maxX = Math.max(...xs)
        const minY = Math.min(...ys)
        const maxY = Math.max(...ys)
        // add 10% padding
        const padX = (maxX - minX) * 0.1 || 0.5
        const padY = (maxY - minY) * 0.1 || 0.5
        return { minX: minX - padX, maxX: maxX + padX, minY: minY - padY, maxY: maxY + padY }
    }, [xs, ys])

    useEffect(() => {
        const canvas = canvasRef.current
        if (!canvas) return
        const ctx = canvas.getContext('2d')
        const W = canvas.width
        const H = canvas.height

        // --- helpers ---
        const toScreen = (wx, wy) => ({
            sx: ((wx - minX) / (maxX - minX)) * W,
            sy: H - ((wy - minY) / (maxY - minY)) * H,
        })

        // Background
        ctx.fillStyle = '#0d1117'
        ctx.fillRect(0, 0, W, H)

        // Grid lines (16×16)
        ctx.strokeStyle = 'rgba(255,255,255,0.06)'
        ctx.lineWidth = 1
        for (let i = 0; i <= 16; i++) {
            const x = (i / 16) * W
            const y = (i / 16) * H
            ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke()
            ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke()
        }

        if (xs.length === 0) return

        const idx = getIndexForTime(csvData.timestamps, currentTimeUs)

        // Full future trail (dim)
        ctx.beginPath()
        for (let i = 0; i < xs.length; i++) {
            const { sx, sy } = toScreen(xs[i], ys[i])
            i === 0 ? ctx.moveTo(sx, sy) : ctx.lineTo(sx, sy)
        }
        ctx.strokeStyle = 'rgba(0, 120, 255, 0.15)'
        ctx.lineWidth = 1.5
        ctx.stroke()

        // Past trail (bright)
        ctx.beginPath()
        for (let i = 0; i <= idx; i++) {
            const { sx, sy } = toScreen(xs[i], ys[i])
            i === 0 ? ctx.moveTo(sx, sy) : ctx.lineTo(sx, sy)
        }
        ctx.strokeStyle = 'rgba(0, 180, 255, 0.85)'
        ctx.lineWidth = 2
        ctx.stroke()

        // Current position dot
        const cur = toScreen(xs[idx], ys[idx])
        const r = Math.min(W, H) / 48
        ctx.beginPath()
        ctx.arc(cur.sx, cur.sy, r, 0, Math.PI * 2)
        ctx.fillStyle = '#00ff88'
        ctx.fill()
        ctx.strokeStyle = '#ffffff'
        ctx.lineWidth = 1.5
        ctx.stroke()

        // Heading arrow
        const theta = thetas[idx] ?? 0
        const arrowLen = r * 2.5
        const ax = cur.sx + Math.cos(theta) * arrowLen
        const ay = cur.sy - Math.sin(theta) * arrowLen  // canvas Y is flipped
        ctx.beginPath()
        ctx.moveTo(cur.sx, cur.sy)
        ctx.lineTo(ax, ay)
        ctx.strokeStyle = '#ff4444'
        ctx.lineWidth = 2.5
        ctx.stroke()

        // Arrowhead
        const angle = Math.atan2(-(ay - cur.sy), ax - cur.sx)
        const hw = 5
        ctx.beginPath()
        ctx.moveTo(ax, ay)
        ctx.lineTo(ax - hw * Math.cos(angle - 0.4), ay + hw * Math.sin(angle - 0.4))
        ctx.lineTo(ax - hw * Math.cos(angle + 0.4), ay + hw * Math.sin(angle + 0.4))
        ctx.closePath()
        ctx.fillStyle = '#ff4444'
        ctx.fill()

    }, [xs, ys, thetas, currentTimeUs, minX, maxX, minY, maxY, csvData])

    // Resize canvas to fill container
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
            </h2>
            {(!colX || !colY) ? (
                <p className="empty-msg">Select X and Y columns in Controls</p>
            ) : (
                <canvas ref={canvasRef} className="grid-canvas" />
            )}
        </div>
    )
}
