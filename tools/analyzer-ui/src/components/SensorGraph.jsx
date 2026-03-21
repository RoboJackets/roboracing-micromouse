import { useMemo, useCallback, useState, useEffect } from 'react'
import {
    LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip,
    ReferenceLine, ResponsiveContainer,
} from 'recharts'

// Palette — cycles if more columns than colours
const PALETTE = [
    '#ff6b6b', '#ffd93d', '#6bcb77', '#4d96ff',
    '#c77dff', '#ff9f43', '#48dbfb', '#ff6b9d',
]

// Returns true for columns that should never appear in the graph
const isExcluded = (h) =>
    h === 'timestamp_us' ||
    h === 'state' ||
    h.startsWith('state/') ||
    h.startsWith('walls[') ||
    h.startsWith('explored[')

/**
 * SensorGraph — configurable time-series chart.
 *
 * All non-excluded CSV columns are available as toggleable lines.
 * Defaults to selecting w/x, w/y, and any IR column on load.
 * Clicking the chart seeks the playback position.
 */
export default function SensorGraph({ csvData, currentTimeUs, onSeek }) {
    // ── Derive all graphable columns ──────────────────────────────────────
    const allCols = useMemo(() => {
        if (!csvData) return []
        return csvData.headers.filter((h) => !isExcluded(h))
    }, [csvData])

    // ── Visibility state — reset when CSV changes ─────────────────────────
    const [visible, setVisible] = useState({})

    useEffect(() => {
        if (!allCols.length) return
        const next = {}
        for (const col of allCols) {
            // Default ON for w/x, w/y, and any IR column; off for everything else
            const isDefault =
                col === 'w/x' ||
                col === 'w/y' ||
                /^ir\[/i.test(col)
            next[col] = isDefault
        }
        setVisible(next)
    }, [allCols])

    const toggleCol = useCallback((col) => {
        setVisible((prev) => ({ ...prev, [col]: !prev[col] }))
    }, [])

    const selectAll = useCallback(() => setVisible(Object.fromEntries(allCols.map(c => [c, true]))), [allCols])
    const selectNone = useCallback(() => setVisible(Object.fromEntries(allCols.map(c => [c, false]))), [allCols])

    const visibleCols = allCols.filter((c) => visible[c])

    // ── Downsample to ≤ 2000 points ───────────────────────────────────────
    const chartData = useMemo(() => {
        if (!csvData || !allCols.length) return []
        const n = csvData.timestamps.length
        const step = Math.max(1, Math.floor(n / 2000))
        const rows = []
        for (let i = 0; i < n; i += step) {
            const row = { t: csvData.timestamps[i] }
            for (const col of allCols) row[col] = csvData.columns[col]?.[i]
            rows.push(row)
        }
        return rows
    }, [csvData, allCols])

    const handleClick = useCallback((data) => {
        if (data?.activePayload?.length) onSeek(data.activeLabel)
    }, [onSeek])

    const fmtTime = (t) => `${(t / 1000).toFixed(0)}ms`

    // ── Empty state ───────────────────────────────────────────────────────
    if (!csvData) {
        return (
            <div className="panel graph-panel">
                <h2 className="panel-title"><span className="icon">📈</span> Graphs</h2>
                <p className="empty-msg">Load a CSV to see data</p>
            </div>
        )
    }

    return (
        <div className="panel graph-panel">
            {/* ── Header + column toggles ── */}
            <div className="graph-header">
                <div className="graph-title-row">
                    <h2 className="panel-title">
                        <span className="icon">📈</span> Graphs
                        <span className="subtitle">&nbsp;— click chart to seek</span>
                    </h2>
                    <div className="graph-sel-btns">
                        <button className="sel-btn" onClick={selectAll}>All</button>
                        <button className="sel-btn" onClick={selectNone}>None</button>
                    </div>
                </div>

                {allCols.length > 0 && (
                    <div className="sensor-toggles">
                        {allCols.map((col, i) => {
                            const on = !!visible[col]
                            const color = PALETTE[i % PALETTE.length]
                            return (
                                <button
                                    key={col}
                                    className={`sensor-chip ${on ? 'active' : 'inactive'}`}
                                    style={{ '--chip-color': color }}
                                    onClick={() => toggleCol(col)}
                                    title={on ? `Hide ${col}` : `Show ${col}`}
                                >
                                    <span className="chip-dot" />
                                    {col}
                                </button>
                            )
                        })}
                    </div>
                )}
            </div>

            {/* ── Chart ── */}
            {visibleCols.length === 0 ? (
                <p className="empty-msg">Select at least one column above</p>
            ) : (
                <ResponsiveContainer width="100%" height="100%">
                    <LineChart
                        data={chartData}
                        margin={{ top: 4, right: 16, left: 0, bottom: 4 }}
                        onClick={handleClick}
                        style={{ cursor: 'crosshair' }}
                    >
                        <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.06)" />
                        <XAxis
                            dataKey="t"
                            type="number"
                            domain={['dataMin', 'dataMax']}
                            tickFormatter={fmtTime}
                            tick={{ fill: '#8899aa', fontSize: 11 }}
                            tickLine={false}
                            axisLine={{ stroke: 'rgba(255,255,255,0.15)' }}
                        />
                        <YAxis
                            tick={{ fill: '#8899aa', fontSize: 11 }}
                            tickLine={false}
                            axisLine={false}
                            width={40}
                        />
                        <Tooltip
                            contentStyle={{
                                background: '#1a2233',
                                border: '1px solid rgba(255,255,255,0.1)',
                                borderRadius: 6,
                                fontSize: 12,
                                color: '#cdd',
                            }}
                            labelFormatter={fmtTime}
                            isAnimationActive={false}
                            formatter={(value, name) =>
                                visible[name] && value != null
                                    ? [Number(value).toFixed(4), name]
                                    : [null, null]
                            }
                        />
                        {allCols.map((col, i) => (
                            <Line
                                key={col}
                                type="monotone"
                                dataKey={col}
                                stroke={PALETTE[i % PALETTE.length]}
                                dot={false}
                                strokeWidth={1.8}
                                isAnimationActive={false}
                                hide={!visible[col]}
                            />
                        ))}
                        <ReferenceLine
                            x={currentTimeUs}
                            stroke="rgba(255,255,255,0.7)"
                            strokeWidth={1.5}
                            strokeDasharray="4 3"
                        />
                    </LineChart>
                </ResponsiveContainer>
            )}
        </div>
    )
}
