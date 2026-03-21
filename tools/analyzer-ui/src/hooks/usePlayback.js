import { useRef, useState, useCallback, useEffect } from 'react'

/**
 * usePlayback — encapsulates all playback state & rAF loop.
 *
 * @param {number[]} timestamps  Array of timestamp_us values from the CSV
 * @returns playback controls and current time
 */
export function usePlayback(timestamps) {
    const [currentTimeUs, setCurrentTimeUs] = useState(0)
    const [playing, setPlaying] = useState(false)
    const [playbackSpeed, setPlaybackSpeed] = useState(1)

    const stateRef = useRef({ playing: false, speed: 1, currentTimeUs: 0, timestamps: [] })
    stateRef.current.timestamps = timestamps
    stateRef.current.playing = playing
    stateRef.current.speed = playbackSpeed
    stateRef.current.currentTimeUs = currentTimeUs

    const rafRef = useRef(null)
    const lastTsRef = useRef(null)

    const tick = useCallback((now) => {
        const s = stateRef.current
        if (!s.playing) return

        if (lastTsRef.current === null) {
            lastTsRef.current = now
        }
        const deltaMs = now - lastTsRef.current
        lastTsRef.current = now

        const maxTime = s.timestamps[s.timestamps.length - 1] ?? 0
        const minTime = s.timestamps[0] ?? 0
        const next = s.currentTimeUs + deltaMs * 1000 * s.speed // ms → us

        if (next >= maxTime) {
            setCurrentTimeUs(maxTime)
            setPlaying(false)
        } else {
            setCurrentTimeUs(Math.max(minTime, next))
            rafRef.current = requestAnimationFrame(tick)
        }
    }, [])

    useEffect(() => {
        if (playing) {
            lastTsRef.current = null
            rafRef.current = requestAnimationFrame(tick)
        } else {
            if (rafRef.current) cancelAnimationFrame(rafRef.current)
        }
        return () => {
            if (rafRef.current) cancelAnimationFrame(rafRef.current)
        }
    }, [playing, tick])

    // Reset when new CSV is loaded
    useEffect(() => {
        if (timestamps.length > 0) {
            setCurrentTimeUs(timestamps[0])
            setPlaying(false)
        }
    }, [timestamps])

    const play = useCallback(() => setPlaying(true), [])
    const pause = useCallback(() => setPlaying(false), [])
    const reset = useCallback(() => {
        setPlaying(false)
        setCurrentTimeUs(timestamps[0] ?? 0)
    }, [timestamps])
    const seekTo = useCallback((t) => setCurrentTimeUs(t), [])

    return { currentTimeUs, playing, playbackSpeed, play, pause, reset, seekTo, setPlaybackSpeed }
}
