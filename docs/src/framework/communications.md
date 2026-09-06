# Communications

## `HardwareInterface`

The trait for external links (UDP today, UART later): `connect`, `send_message`, `listen`, `disconnect`. `encode_frame` / `decode_frame` implement the `[MessageType][protobuf]` wire format described in [Messages](messages.md). Internal-only variants (`MotorCommands`, `VehicleWrench`) do not encode and yield an **empty** frame.

## `UdpInterface`

<figure>
<svg class="diagram" viewBox="0 0 700 210" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="UDP interface: a receive thread publishing decoded frames on a channel and a transmit thread consuming that channel">
  <defs>
    <marker id="c-arw" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
  </defs>
  <rect x="40" y="40" width="120" height="120" rx="6" fill="currentColor" fill-opacity="0.05" stroke="currentColor" stroke-opacity="0.4"/>
  <text x="100" y="66" text-anchor="middle" font-size="12" font-weight="600">Remote peer</text>
  <text x="100" y="86" text-anchor="middle" font-size="10.5" opacity="0.75">UDP socket</text>
  <rect x="250" y="30" width="180" height="58" rx="5" fill="currentColor" fill-opacity="0.06" stroke="#2aa198" stroke-width="1.2"/>
  <text x="340" y="53" text-anchor="middle" font-size="12" font-weight="600">RX thread</text>
  <text x="340" y="72" text-anchor="middle" font-size="10.5" opacity="0.75">blocking recv_from → decode</text>
  <rect x="250" y="118" width="180" height="58" rx="5" fill="currentColor" fill-opacity="0.06" stroke="#cb8b1e" stroke-width="1.2"/>
  <text x="340" y="141" text-anchor="middle" font-size="12" font-weight="600">TX thread</text>
  <text x="340" y="160" text-anchor="middle" font-size="10.5" opacity="0.75">blocking_recv → encode → send</text>
  <rect x="510" y="70" width="150" height="66" rx="5" fill="currentColor" fill-opacity="0.06" stroke="#4a90d9" stroke-width="1.2"/>
  <text x="585" y="97" text-anchor="middle" font-size="12" font-weight="600">broadcast</text>
  <text x="585" y="115" text-anchor="middle" font-size="12" font-weight="600">channel</text>
  <line x1="160" y1="60" x2="244" y2="60" stroke="currentColor" stroke-width="1.3" marker-end="url(#c-arw)"/>
  <line x1="430" y1="60" x2="560" y2="64" stroke="currentColor" stroke-width="1.3" marker-end="url(#c-arw)"/>
  <line x1="560" y1="142" x2="436" y2="146" stroke="currentColor" stroke-width="1.3" marker-end="url(#c-arw)"/>
  <line x1="244" y1="146" x2="164" y2="146" stroke="currentColor" stroke-width="1.3" marker-end="url(#c-arw)"/>
  <text x="100" y="180" text-anchor="middle" font-size="10.5" opacity="0.7">poison pill on disconnect</text>
  <path d="M100 160 L100 172" fill="none" stroke="currentColor" stroke-opacity="0.5" stroke-width="1" stroke-dasharray="3 3"/>
</svg>
<figcaption>Two dedicated threads; the interface is not a scheduled process.</figcaption>
</figure>

`connect()` binds the source socket, resolves the destination, then spawns:

- an **RX thread**: blocking `recv_from`, every decoded frame published on the broadcast channel. It is unblocked by an empty datagram (a "poison pill") sent by `disconnect()`;
- a **TX thread**: reads the channel through `blocking_recv()` and sends each message to the destination.

`Process::exec` is `todo!()`: the interface **is not scheduled**, it lives on its threads.

> **Known fragilities**, flagged as `// NOTE:` in the source:
> - the TX loop `while let Ok(msg) = blocking_recv()` **exits permanently on the first `Lagged`** (a slow receiver on a small-capacity channel). `Lagged` means *"messages were skipped, carry on"*, never *"stop"*;
> - internal-only variants encode to an **empty** frame, which is exactly the RX thread's poison pill;
> - RX and TX wired on the same channel echo received frames back out;
> - `disconnect()`'s join can block while other senders are alive.

## Remote control

`XboxPadControl` (gilrs) is a `Process`: on each `exec`, `scan_buttons` polls up to `n_listening` events (`next_event()` is **non-blocking**) and returns a `RemoteControl` — a snapshot of the sticks and buttons *seen this tick*. It is registered in the main chain, but its output is not consumed by the control chain yet: stabilize mode uses a fixed identity setpoint. Wiring it later means building `q_d` from the sticks (roll and pitch directly, yaw integrated to hold a heading) — pure setpoint construction, no new theory.

> `button_pressed` is set on a **release** event, `stick_pressed` receives a trigger-axis id rather than a stick click, and a tick with no event returns an all-zero snapshot rather than the last state.
