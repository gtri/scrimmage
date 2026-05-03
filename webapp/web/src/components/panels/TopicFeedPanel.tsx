import { useEffect, useMemo, useState } from 'react';
import { useTopicStream } from '../../hooks/useTopicStream';
import type { TopicMessage } from '../../types';

type Formatter = (msg: TopicMessage) => string;

const FORMATTERS: Record<string, Formatter> = {
  'scrimmage_msgs.CaptureEntity': (msg) => {
    try {
      const p = JSON.parse(msg.payloadJson);
      return `t=${msg.tSim.toFixed(2)}s — entity ${p.sourceId} captured entity ${p.targetId}`;
    } catch {
      return `t=${msg.tSim.toFixed(2)}s — ${msg.payloadJson}`;
    }
  },
};

function formatMessage(msg: TopicMessage): string {
  const fmt = FORMATTERS[msg.typeName];
  if (fmt) return fmt(msg);
  try {
    const p = JSON.parse(msg.payloadJson);
    return `t=${msg.tSim.toFixed(2)}s — ${JSON.stringify(p)}`;
  } catch {
    return `t=${msg.tSim.toFixed(2)}s — ${msg.payloadJson}`;
  }
}

function PanelTitle({ children }: { children: React.ReactNode }) {
  return (
    <div style={{
      fontSize: 10, fontWeight: 700, letterSpacing: '0.2em',
      textTransform: 'uppercase', color: 'var(--accent)',
      marginBottom: 6,
      display: 'flex', alignItems: 'center', gap: 6,
    }}>
      <span style={{ width: 8, height: 1, background: 'var(--accent)', display: 'inline-block' }} />
      {children}
    </div>
  );
}

export function TopicFeedPanel() {
  const { topics, messagesFor } = useTopicStream();
  const [selected, setSelected] = useState<string>('');

  useEffect(() => {
    if (topics.length === 0) {
      setSelected('');
    } else if (!topics.some(t => `${t.network}:${t.topic}` === selected)) {
      setSelected(`${topics[0].network}:${topics[0].topic}`);
    }
  }, [topics, selected]);

  const messages = useMemo(() => {
    if (!selected) return [];
    const [network, topic] = selected.split(':', 2);
    return messagesFor(network, topic);
  }, [selected, messagesFor]);

  return (
    <div style={{
      padding: '12px 14px',
      borderBottom: '1px solid var(--border-default)',
    }}>
      <PanelTitle>Topics</PanelTitle>

      {topics.length === 0 ? (
        <div style={{ fontSize: 11, lineHeight: 1.5, color: 'var(--text-muted)' }}>
          No topics configured for this mission.
        </div>
      ) : (
        <>
          <select
            value={selected}
            onChange={e => setSelected(e.target.value)}
            style={{
              width: '100%',
              marginBottom: 8,
              background: 'var(--bg-base)',
              color: 'var(--text-primary)',
              border: '1px solid var(--border-default)',
              borderRadius: 2,
              fontSize: 11,
              padding: '3px 6px',
            }}
          >
            {topics.map(t => {
              const k = `${t.network}:${t.topic}`;
              return (
                <option key={k} value={k}>{t.network} / {t.topic}</option>
              );
            })}
          </select>

          <ul style={{
            listStyle: 'none',
            margin: 0,
            padding: 0,
            maxHeight: 240,
            overflowY: 'auto',
          }}>
            {messages.length === 0 ? (
              <li style={{ fontSize: 11, lineHeight: 1.5, color: 'var(--text-muted)' }}>
                Waiting for messages on {selected}…
              </li>
            ) : (
              messages.map((msg, i) => (
                <li
                  key={`${msg.tSim}-${i}`}
                  style={{
                    fontSize: 11,
                    lineHeight: 1.6,
                    color: 'var(--text-secondary)',
                    fontFamily: 'var(--font-mono)',
                    borderBottom: '1px solid var(--border-default)',
                    padding: '2px 0',
                  }}
                >
                  {formatMessage(msg)}
                </li>
              ))
            )}
          </ul>
        </>
      )}
    </div>
  );
}
