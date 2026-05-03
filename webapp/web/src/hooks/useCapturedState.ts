import { useEffect, useRef, useState } from 'react';
import type { EntityDto, FrameDto } from '../types';

export type InspectorMode = 'idle' | 'live' | 'captured';

export interface InspectorState {
  mode: InspectorMode;
  entity: EntityDto | null;
  killerId: number | null;
}

const CAPTURED_HOLD_MS = 2000;

/**
 * Drives the inspector card's mode based on the current selection and frame.
 *
 * - idle: no selection.
 * - live: selected entity is present in the latest frame; entity snapshot updates each tick.
 * - captured: selection was live in the prior frame but missing from the latest frame.
 *   Killer ID is the nearest non-team entity from the prior frame. After
 *   CAPTURED_HOLD_MS, onAutoClear fires so the parent can clear the selection.
 */
export function useCapturedState(
  selectedEntityId: number | null,
  latestFrame: FrameDto | null,
  onAutoClear: () => void,
): InspectorState {
  const [state, setState] = useState<InspectorState>({
    mode: 'idle',
    entity: null,
    killerId: null,
  });
  const prevFrameRef = useRef<FrameDto | null>(null);
  const timerRef = useRef<number | null>(null);
  // Keep the latest onAutoClear in a ref to avoid stale closures inside setTimeout.
  const onAutoClearRef = useRef(onAutoClear);
  onAutoClearRef.current = onAutoClear;

  useEffect(() => {
    if (selectedEntityId == null) {
      if (timerRef.current != null) {
        window.clearTimeout(timerRef.current);
        timerRef.current = null;
      }
      setState({ mode: 'idle', entity: null, killerId: null });
      prevFrameRef.current = latestFrame;
      return;
    }

    if (latestFrame == null) {
      prevFrameRef.current = latestFrame;
      return;
    }

    const current = latestFrame.entities.find(e => e.id === selectedEntityId);

    if (current) {
      // Selection cleared a pending captured timer if any (e.g., user picked a
      // new drone during the 2 s window).
      if (timerRef.current != null) {
        window.clearTimeout(timerRef.current);
        timerRef.current = null;
      }
      setState({ mode: 'live', entity: current, killerId: null });
    } else {
      const prev = prevFrameRef.current?.entities.find(
        e => e.id === selectedEntityId,
      );
      if (prev) {
        // Just transitioned. Find nearest enemy in prior frame.
        const prevEntities = prevFrameRef.current?.entities ?? [];
        let killerId: number | null = null;
        let bestSq = Infinity;
        for (const c of prevEntities) {
          if (c.teamId === prev.teamId) continue;
          if (c.id === prev.id) continue;
          const dx = c.position.x - prev.position.x;
          const dy = c.position.y - prev.position.y;
          const dz = c.position.z - prev.position.z;
          const sq = dx * dx + dy * dy + dz * dz;
          if (sq < bestSq) { bestSq = sq; killerId = c.id; }
        }
        setState({ mode: 'captured', entity: prev, killerId });
        if (timerRef.current != null) window.clearTimeout(timerRef.current);
        timerRef.current = window.setTimeout(() => {
          timerRef.current = null;
          onAutoClearRef.current();
        }, CAPTURED_HOLD_MS);
      }
      // else: not present in either frame; stay in current mode.
    }

    prevFrameRef.current = latestFrame;
  }, [latestFrame, selectedEntityId]);

  // Cleanup on unmount.
  useEffect(() => {
    return () => {
      if (timerRef.current != null) {
        window.clearTimeout(timerRef.current);
        timerRef.current = null;
      }
    };
  }, []);

  return state;
}
