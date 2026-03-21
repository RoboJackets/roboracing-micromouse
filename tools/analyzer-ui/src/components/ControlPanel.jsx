import { useRef, useCallback } from 'react'

/**
 * ControlPanel — CSV loader, playback controls, column selectors, timeline.
 */
export default function ControlPanel({
    csvData,
    onLoadCSV,
    currentTimeUs,
    playing,
    playbackSpeed,
    onPlay,
    onPause,
    onReset,
    onSeek,
    onSpeedChange,
    colX, onColXChange,
    colY, onColYChange,
    colTheta, onColThetaChange,
}) {
    const fileInputRef = useRef(null)
    const dropRef = useRef(null)

    const handleFile = useCallback((file) => {
        if (!file) return
        const reader = new FileReader()
        reader.onload = (e) => onLoadCSV(e.target.result, file.name)
        reader.readAsText(file)
    }, [onLoadCSV])

    const handleInputChange = (e) => handleFile(e.target.files[0])

    const handleDrop = (e) => {
        e.preventDefault()
        dropRef.current?.classList.remove('drag-over')
        handleFile(e.dataTransfer.files[0])
    }

    const handleDragOver = (e) => {
        e.preventDefault()
        dropRef.current?.classList.add('drag-over')
    }

    const handleDragLeave = () => dropRef.current?.classList.remove('drag-over')

    const selectableHeaders = csvData
        ? csvData.headers.filter((h) => h !== 'timestamp_us')
        : []

    const minTime = csvData?.timestamps[0] ?? 0
    const maxTime = csvData?.timestamps[csvData.timestamps.length - 1] ?? 1
    const timeMs = Math.round(currentTimeUs / 1000)
    const maxMs = Math.round(maxTime / 1000)

    const ColSelect = ({ label, value, onChange }) => (
        <div className="form-row">
            <label>{label}</label>
            <select value={value} onChange={(e) => onChange(e.target.value)}>
                <option value="">None</option>
                {selectableHeaders.map((h) => (
                    <option key={h} value={h}>{h}</option>
                ))}
            </select>
        </div>
    )

    return (
        <div className="panel control-panel">
            <h2 className="panel-title">
                <span className="icon">⚙</span> Controls
            </h2>

            {/* Drop zone */}
            <div
                ref={dropRef}
                className="drop-zone"
                onClick={() => fileInputRef.current?.click()}
                onDrop={handleDrop}
                onDragOver={handleDragOver}
                onDragLeave={handleDragLeave}
            >
                <span className="drop-icon">📁</span>
                <span>{csvData ? `Loaded: ${csvData.filename}` : 'Drop CSV here or click to browse'}</span>
                <input
                    ref={fileInputRef}
                    type="file"
                    accept=".csv"
                    style={{ display: 'none' }}
                    onChange={handleInputChange}
                />
            </div>

            {csvData && (
                <>
                    <div className="separator" />

                    {/* Playback row */}
                    <div className="playback-row">
                        <button
                            id="btn-play-pause"
                            className={`btn ${playing ? 'btn-pause' : 'btn-play'}`}
                            onClick={playing ? onPause : onPlay}
                        >
                            {playing ? '⏸ Pause' : '▶ Play'}
                        </button>
                        <button id="btn-reset" className="btn btn-secondary" onClick={onReset}>
                            ↺ Reset
                        </button>
                        <span className="time-display">
                            {timeMs.toLocaleString()} / {maxMs.toLocaleString()} ms
                        </span>
                    </div>

                    {/* Timeline */}
                    <div className="form-row">
                        <label>Timeline</label>
                        <input
                            id="timeline-slider"
                            type="range"
                            min={minTime}
                            max={maxTime}
                            step={(maxTime - minTime) / 2000}
                            value={currentTimeUs}
                            onChange={(e) => onSeek(Number(e.target.value))}
                            className="slider"
                        />
                    </div>

                    {/* Speed */}
                    <div className="form-row">
                        <label>Speed&nbsp;<strong>{playbackSpeed.toFixed(1)}×</strong></label>
                        <input
                            id="speed-slider"
                            type="range"
                            min="0.1"
                            max="10"
                            step="0.1"
                            value={playbackSpeed}
                            onChange={(e) => onSpeedChange(Number(e.target.value))}
                            className="slider"
                        />
                    </div>

                    <div className="separator" />
                    <p className="section-label">2D Grid Mappings</p>

                    <ColSelect label="X Coord" value={colX} onChange={onColXChange} />
                    <ColSelect label="Y Coord" value={colY} onChange={onColYChange} />
                    <ColSelect label="Θ Heading" value={colTheta} onChange={onColThetaChange} />

                    <div className="separator" />
                    <div className="meta-row">
                        <span>{csvData.timestamps.length.toLocaleString()} frames</span>
                        <span>{csvData.headers.length} columns</span>
                    </div>
                </>
            )}
        </div>
    )
}
