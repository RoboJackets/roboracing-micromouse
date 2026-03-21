import { useMemo, useCallback } from 'react'
import {
    LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip,
    ReferenceLine, Legend, ResponsiveContainer,
} from 'recharts'

const IR_COLORS = ['#ff6b6b', '#ffd93d', '#6bcb77', '#4d96ff']

/**
 * SensorGraph — time-series chart for all IR columns.
 * Shows a vertical reference line at the current playback time.
 * Clicking the chart seeks to that time.
 */
export default function SensorGraph({ csvData, currentTimeUs, onSeek }) {
    // Collect all IR column names
    const irCols = useMemo(() => {
        if (!csvData) return []
        return csvData.headers.filter(
            (h) => h.toLowerCase().includes('ir') ||
                h.toLowerCase().includes('front') ||
                h.toLowerCase().includes('side')
        )
    }, [csvData])

    // Downsample to at most 2000 points (for large CSVs) to keep rendering fast
    const chartData = useMemo(() => {
        if (!csvData || irCols.length === 0) return []
        const n = csvData.timestamps.length
        const step = Math.max(1, Math.floor(n / 2000))
        const rows = []
        for (let i = 0; i < n; i += step) {
            const row = { t: csvData.timestamps[i] }
            for (const col of irCols) row[col] = csvData.columns[col][i]
            rows.push(row)
        }
        return rows
    }, [csvData, irCols])

    const handleClick = useCallback((data) => {
        if (data?.activePayload?.length) {
            onSeek(data.activeLabel)
        }
    }, [onSeek])

    const formatTime = (t) => `${(t / 1000).toFixed(0)}ms`

    if (!csvData) {
        return (
            <div className="panel graph-panel">
                <h2 className="panel-title"><span className="icon">📈</span> Sensor Graphs</h2>
                <p className="empty-msg">Load a CSV to see sensor data</p>
            </div>
        )
    }

    return (
        <div className="panel graph-panel">
            <h2 className="panel-title">
                <span className="icon">📈</span> Sensor Graphs
                <span className="subtitle">&nbsp;— click to seek</span>
            </h2>
            {irCols.length === 0 ? (
                <p className="empty-msg">No IR / sensor columns found</p>
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
                            tickFormatter={formatTime}
                            tick={{ fill: '#8899aa', fontSize: 11 }}
                            tickLine={false}
                            axisLine={{ stroke: 'rgba(255,255,255,0.15)' }}
                        />
                        <YAxis
                            tick={{ fill: '#8899aa', fontSize: 11 }}
                            tickLine={false}
                            axisLine={false}
                            width={36}
                        />
                        <Tooltip
                            contentStyle={{
                                background: '#1a2233',
                                border: '1px solid rgba(255,255,255,0.1)',
                                borderRadius: 6,
                                fontSize: 12,
                                color: '#cdd',
                            }}
                            labelFormatter={formatTime}
                            isAnimationActive={false}
                        />
                        <Legend
                            wrapperStyle={{ fontSize: 12, color: '#8899aa', paddingTop: 4 }}
                        />
                        {irCols.map((col, i) => (
                            <Line
                                key={col}
                                type="monotone"
                                dataKey={col}
                                stroke={IR_COLORS[i % IR_COLORS.length]}
                                dot={false}
                                strokeWidth={1.8}
                                isAnimationActive={false}
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
