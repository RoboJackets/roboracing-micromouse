import { useMemo } from 'react'
import { getIndexForTime } from '../utils/csvParser'

/**
 * StateTable — shows all column values at the current frame index.
 * Mirrors Dashboard::renderStateWindow().
 */
export default function StateTable({ csvData, currentTimeUs }) {
    const idx = useMemo(
        () => getIndexForTime(csvData?.timestamps ?? [], currentTimeUs),
        [csvData, currentTimeUs]
    )

    if (!csvData) return null

    const nonTimestamp = csvData.headers.filter(
        (h) =>
            h !== 'timestamp_us' &&
            !h.startsWith('walls[') &&
            !h.startsWith('explored[')
    )

    return (
        <div className="panel state-panel">
            <h2 className="panel-title">
                <span className="icon">📋</span> Robot State
                <span className="subtitle">&nbsp;@ {Math.round(currentTimeUs / 1000)} ms</span>
            </h2>
            <div className="state-table-wrap">
                <table className="state-table">
                    <thead>
                        <tr>
                            <th>Variable</th>
                            <th>Value</th>
                        </tr>
                    </thead>
                    <tbody>
                        {nonTimestamp.map((col) => (
                            <tr key={col}>
                                <td className="col-name">{col}</td>
                                <td className="col-val">
                                    {(csvData.columns[col][idx] ?? 0).toFixed(4)}
                                </td>
                            </tr>
                        ))}
                    </tbody>
                </table>
            </div>
        </div>
    )
}
