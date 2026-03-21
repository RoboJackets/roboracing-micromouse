/**
 * parseCSV — parse a raw CSV string into structured telemetry data.
 * Mirrors the C++ CSVData struct.
 *
 * @param {string} text  Raw CSV file content
 * @returns {{
 *   headers: string[],
 *   timestamps: number[],
 *   columns: Record<string, number[]>,
 *   colX: string,
 *   colY: string,
 *   colTheta: string,
 * }}
 */
export function parseCSV(text) {
    const lines = text.replace(/\r/g, '').split('\n').filter(Boolean)
    if (lines.length < 2) throw new Error('CSV has no data rows')

    const headers = lines[0].split(',').map((h) => h.trim())

    const columns = {}
    for (const h of headers) columns[h] = []

    for (let i = 1; i < lines.length; i++) {
        const values = lines[i].split(',')
        for (let j = 0; j < headers.length; j++) {
            columns[headers[j]].push(parseFloat(values[j] ?? 'NaN'))
        }
    }

    const timestamps = columns['timestamp_us'] ?? []

    // Auto-detect world coordinate columns (same logic as Dashboard.cpp)
    let colX = ''
    let colY = ''
    let colTheta = ''
    for (const h of headers) {
        if (!colX && h.includes('/x')) colX = h
        if (!colY && h.includes('/y')) colY = h
        if (!colTheta && h.includes('/theta')) colTheta = h
    }

    return { headers, timestamps, columns, colX, colY, colTheta }
}

/**
 * Binary-search for the data row index whose timestamp ≤ timeUs.
 * Mirrors Dashboard::getIndexForTime().
 */
export function getIndexForTime(timestamps, timeUs) {
    if (!timestamps || timestamps.length === 0) return 0
    let lo = 0
    let hi = timestamps.length - 1
    while (lo < hi) {
        const mid = (lo + hi + 1) >> 1
        if (timestamps[mid] <= timeUs) lo = mid
        else hi = mid - 1
    }
    return lo
}
