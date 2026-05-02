import * as signalR from '@microsoft/signalr';
import { useEffect, useRef, useState } from 'react';
import type { FrameDto } from '../types';

const API_URL = (import.meta as any).env.VITE_API_URL as string ?? 'http://localhost:8080';

type FrameHandler = (frame: FrameDto) => void;

/**
 * Subscribes to /hubs/frames. The `onFrame` callback fires on every frame (no React re-render).
 * Also exposes a throttled `latestFrame` state that updates ~5 Hz, suitable for sidebar UI.
 */
export function useFrameStream(onFrame: FrameHandler) {
  const [connected, setConnected] = useState(false);
  const [latestFrame, setLatestFrame] = useState<FrameDto | null>(null);
  const lastUiUpdate = useRef(0);
  const handlerRef = useRef(onFrame);
  handlerRef.current = onFrame;

  useEffect(() => {
    const conn = new signalR.HubConnectionBuilder()
      .withUrl(`${API_URL}/hubs/frames`)
      .withAutomaticReconnect()
      .build();

    conn.on('OnFrame', (frame: FrameDto) => {
      handlerRef.current(frame);
      const now = performance.now();
      if (now - lastUiUpdate.current > 200) {
        lastUiUpdate.current = now;
        setLatestFrame(frame);
      }
    });

    conn.onreconnected(() => setConnected(true));
    conn.onclose(() => setConnected(false));

    conn.start()
      .then(() => setConnected(true))
      .catch(err => console.error('SignalR connect failed:', err));

    return () => { conn.stop(); };
  }, []);

  return { connected, latestFrame };
}
