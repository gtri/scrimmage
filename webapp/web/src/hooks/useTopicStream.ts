import * as signalR from '@microsoft/signalr';
import { useEffect, useRef, useState } from 'react';
import type { TopicSpec, TopicMessage } from '../types';

const API_URL = (import.meta as any).env.VITE_API_URL as string ?? 'http://localhost:8080';

const MAX_MESSAGES_PER_TOPIC = 100;

const key = (network: string, topic: string) => `${network}:${topic}`;

export interface TopicState {
  topics: TopicSpec[];
  messagesByKey: Map<string, TopicMessage[]>;
}

/**
 * Subscribes to /hubs/topics.
 * Exposes the current topic list and a helper to retrieve buffered messages
 * for a specific (network, topic) pair (newest-first, capped at MAX_MESSAGES_PER_TOPIC).
 */
export function useTopicStream() {
  const [state, setState] = useState<TopicState>({
    topics: [],
    messagesByKey: new Map(),
  });
  const stateRef = useRef<TopicState>(state);

  useEffect(() => {
    const conn = new signalR.HubConnectionBuilder()
      .withUrl(`${API_URL}/hubs/topics`)
      .withAutomaticReconnect()
      .build();

    conn.on('OnTopicList', (topics: TopicSpec[]) => {
      // New session boundary: reset message feeds.
      const next: TopicState = { topics, messagesByKey: new Map() };
      stateRef.current = next;
      setState(next);
    });

    conn.on('OnTopicMessage', (msg: TopicMessage) => {
      const k = key(msg.network, msg.topic);
      const existing = stateRef.current.messagesByKey.get(k) ?? [];
      const msgs = [msg, ...existing].slice(0, MAX_MESSAGES_PER_TOPIC);
      const newMap = new Map(stateRef.current.messagesByKey);
      newMap.set(k, msgs);
      const next: TopicState = { ...stateRef.current, messagesByKey: newMap };
      stateRef.current = next;
      setState(next);
    });

    conn.start().catch(err => console.error('TopicHub connect failed:', err));

    return () => { conn.stop(); };
  }, []);

  return {
    topics: state.topics,
    messagesFor: (network: string, topic: string): TopicMessage[] =>
      state.messagesByKey.get(key(network, topic)) ?? [],
  };
}
