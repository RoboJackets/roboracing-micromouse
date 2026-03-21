import { useState, useCallback } from 'react'
import { parseCSV } from './utils/csvParser'
import { usePlayback } from './hooks/usePlayback'
import ControlPanel from './components/ControlPanel'
import GridCanvas from './components/GridCanvas'
import SensorGraph from './components/SensorGraph'
import StateTable from './components/StateTable'

export default function App() {
    const [csvData, setCsvData] = useState(null)
    const [colX, setColX] = useState('')
    const [colY, setColY] = useState('')
    const [colTheta, setColTheta] = useState('')

    const {
        currentTimeUs, playing, playbackSpeed,
        play, pause, reset, seekTo, setPlaybackSpeed,
    } = usePlayback(csvData?.timestamps ?? [])

    const handleLoadCSV = useCallback((text, filename) => {
        try {
            const parsed = parseCSV(text)
            parsed.filename = filename
            setCsvData(parsed)
            setColX(parsed.colX)
            setColY(parsed.colY)
            setColTheta(parsed.colTheta)
        } catch (err) {
            alert(`Failed to parse CSV: ${err.message}`)
        }
    }, [])

    return (
        <div className="app-layout">
            {/* Header */}
            <header className="app-header">
                <div className="header-inner">
                    <span className="logo">🐭</span>
                    <h1>Micromouse Telemetry Analyzer</h1>
                    {csvData && (
                        <span className="badge">
                            {csvData.timestamps.length.toLocaleString()} frames
                        </span>
                    )}
                </div>
            </header>

            {/* Main */}
            <main className="main-grid">
                {/* Left column */}
                <aside className="left-col">
                    <ControlPanel
                        csvData={csvData}
                        onLoadCSV={handleLoadCSV}
                        currentTimeUs={currentTimeUs}
                        playing={playing}
                        playbackSpeed={playbackSpeed}
                        onPlay={play}
                        onPause={pause}
                        onReset={reset}
                        onSeek={seekTo}
                        onSpeedChange={setPlaybackSpeed}
                        colX={colX} onColXChange={setColX}
                        colY={colY} onColYChange={setColY}
                        colTheta={colTheta} onColThetaChange={setColTheta}
                    />
                    <StateTable csvData={csvData} currentTimeUs={currentTimeUs} />
                </aside>

                {/* Right column */}
                <section className="right-col">
                    <GridCanvas
                        csvData={csvData}
                        colX={colX}
                        colY={colY}
                        colTheta={colTheta}
                        currentTimeUs={currentTimeUs}
                    />
                    <SensorGraph
                        csvData={csvData}
                        currentTimeUs={currentTimeUs}
                        onSeek={seekTo}
                    />
                </section>
            </main>
        </div>
    )
}
