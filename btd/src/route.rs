//! Which calls BLE may make, and which of a service's connections carries them.
//!
//! BLE exposes a **subset** of the robot API: provisioning, status,
//! and the update commands with their progress. It is too slow and too constrained for the full
//! surface, and — more to the point — a radio anybody within a few metres can talk to is not
//! the transport over which to offer "reset this robot to factory state".
//!
//! **Two questions, and only one of them is BLE's.** *Which service answers a call, and how long
//! answering holds a connection* is a property of the call, and lives in
//! [`proto::Call::destination`] where every transport reads the same answer. *Whether BLE may make
//! it* is this file. They were one table until a second transport needed the first half and none
//! of the second.
//!
//! **The permission match is deliberately exhaustive.** Adding a variant to [`proto::Call`] makes
//! this file fail to compile, so a new method cannot reach the radio because someone forgot this
//! file existed. A `_ => false` wildcard would be the safe default in the moment and the wrong one
//! over time: it would silently deny new methods, and the first symptom would be a phone app that
//! cannot see a feature nobody remembered to route. Every transport needs its own such match for
//! the same reason — a shared one with a wildcard would be the hole in all of them at once.

use duck_ipc_proto as proto;

/// How long a call holds a connection. Defined once, in the protocol crate.
pub use proto::Lane;

/// The service that owns the answer to a call, restricted to the three `btd` holds sockets to.
///
/// Narrower than [`proto::Service`] on purpose. `padd` and `tofd` answer calls too, and `btd` has
/// no connection to either: `padd` is the unprivileged client whose whole value is having no
/// special access, and giving the BLE transport a socket to it would be the first thing to make
/// that untrue. The conversion below therefore *fails* for them, which turns a comment into
/// something the compiler enforces.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Upstream {
    /// `updaterd`, at `proto::DEFAULT_SOCKET`.
    Updater,
    /// `robotd`.
    Robot,
    /// `configd` — wifi and the robot's identity.
    Config,
}

impl TryFrom<proto::Service> for Upstream {
    type Error = ();

    fn try_from(service: proto::Service) -> Result<Self, Self::Error> {
        match service {
            proto::Service::Updater => Ok(Upstream::Updater),
            proto::Service::Robot => Ok(Upstream::Robot),
            proto::Service::Config => Ok(Upstream::Config),
            // Not sockets `btd` holds. Unreachable in practice, because `permits` refuses every
            // call these answer — and an error rather than a panic so that staying true is not
            // something this file has to be careful about.
            proto::Service::Pad | proto::Service::Tof => Err(()),
        }
    }
}

/// What happens to a call that arrives over BLE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Route {
    /// Forwarded verbatim to a service, on that service's connection for this lane.
    To(Upstream, Lane),
    /// Answered by `btd` itself. Only `system.authenticate`: the PIN check belongs to the
    /// transport, because BLE cannot express a fixed printed passkey and the check therefore had
    /// to move up a layer.
    Local,
    /// Not available over this transport.
    Refused,
}

/// May a call arriving over BLE be served at all?
///
/// Read the `false` arms as the security boundary: each one is a deliberate decision that a
/// phone in the room does not get to do this.
fn permits(call: &proto::Call) -> bool {
    use proto::Call::*;
    match call {
        // The version handshake. Must be reachable or no client can establish anything.
        Hello(_) => true,

        // Answered by `btd` itself rather than forwarded. Permitted because it is the one call a
        // session must be able to make before it has made any other.
        SystemAuthenticate(_) => true,

        // ── the update subset §4.1 names ────────────────────────────────────
        //
        // `Apply` is intended: BLE implies physical presence plus pairing (§4.2), and "update
        // the robot from the phone" is M6's headline. It also has to pass `updaterd`'s own peer
        // policy, and does — `deploy/updater.toml` names `btd` in `allow_users`, which is a
        // narrower claim than granting the robot group. Routing it here without that grant would
        // have produced a phone button that always returned PERMISSION_DENIED.
        Apply(_) => true,
        Check(_) => true,
        Status => true,
        Subscribe => true,
        // Read-only, and what support asks for first. `update.log` is the record that
        // survives a wiped journal (§8.2), so a phone able to read it is worth having.
        Log(_) => true,
        // The detail behind one of those log lines, and the same claim: read-only, and the
        // question an owner whose update failed actually has. Bigger than every other reply here
        // — a few kilobytes for an ordinary run, since `hooks::MAX_OUTPUT` bounds the largest
        // part of it, against a `updater::transcript` ceiling of two megabytes for a pathological
        // one — so an app should ask for it on a tap rather than on a refresh. That is the app's
        // decision to make: what this function decides is whether a phone in the room may see it,
        // and a phone that may read the log may read what is behind it.
        Show(_) => true,
        ListInstalled(_) => true,

        // Going back. Both are permitted, and both are less consequential than the `Apply`
        // above them: they move the robot to a release that has already run on this board,
        // download nothing, and are gated and auto-reverted like any other transition
        // (`Engine::rollback` and `Engine::select` both go through `transition_to`).
        //
        // They were refused until the update path was driven from a phone, on the reasoning that
        // the engine reverts a bad release on its own. It does — the one that fails its health
        // gate. That is not the case an owner reaches for a phone about, which is a release that
        // installs, passes its gate, and then behaves *worse*: a policy that walks unsteadily
        // rather than not at all, a pad that stops reconnecting. Nothing reverts that but a
        // person, and the person is holding a phone and has no ssh.
        //
        // `Rollback` is the undo — the previous release, no arguments, one tap. `Select` is the
        // same authority plus a version number, and it is what a list of installed releases is
        // *for*: `ListInstalled` is already routed above, so an app can show them, and being able
        // to show them without being able to choose one would be the odd half.
        Rollback(_) => true,
        Select(_) => true,

        // Is the robot alright? The one `robot.*` call an app has any use for.
        RobotHealth => true,

        // ── provisioning, which is what §4.1 puts BLE here for ──────────────
        //
        // This is the case the whole transport exists to serve: a robot that has never seen a
        // network cannot be configured over that network, so BLE is the only way in. All four
        // are permitted, including the two that change things.
        NetStatus => true,
        NetScan => true,
        // Carries a wifi passphrase, and this arm used to claim that travels over a paired,
        // authenticated link. It does not, and the claim was wrong in both halves.
        //
        // The characteristic sets `encrypt_write`, not `encrypt_authenticated_write`
        // (`crate::bluez`) — and it sets it from `--require-pairing`, which is **off by default**,
        // so on an ordinary board there is no encryption on this link at all. Nor could the
        // stronger flag be satisfied if it were set: the agent leaves every handler `None`, which
        // BlueZ publishes as `NoInputNoOutput`, so the bond is just-works and therefore encrypted
        // but *unauthenticated*. `crate::pairing` records why a headless robot cannot do better,
        //
        // So what actually stands behind this route today is the PIN check in `crate::session` and
        // the ten metres of radio range, and the passphrase crosses in clear. That is a known,
        // accepted, pre-shipping cost — `btd` warns about it at every start — and §8.1 is the
        // blocker that has to close before a robot goes to anyone. Routed anyway because a robot
        // with no network cannot be given one any other way, which is what this transport is for.
        NetConnect(_) => true,
        NetForget(_) => true,

        // Name and identity. Renaming from the app is the reason `system.setName` exists.
        SystemInfo => true,
        SystemSetName(_) => true,

        // Which daemons are up and which release each is running. Routed because an app that can
        // trigger an update should be able to show whether it took — and because the one daemon it
        // cannot report on this way is `btd` itself, which answering at all proves is running.
        SystemServices => true,

        // The tail of one daemon's journal, and the next question after the line above: a unit
        // reported as `failed` is a diagnosis nobody can act on without the reason it failed.
        //
        // Permitted because BLE is where the question is asked. A robot with no network cannot be
        // reached by ssh, and that robot — one whose wifi never came up, whose `robotd` died on
        // boot — is exactly the one whose journal somebody needs. Refusing here would mean the
        // logs are readable over every transport except the one available when things are broken.
        //
        // Read-only, and bounded on the other side rather than trusted: `configd` picks the unit
        // from a fixed list and refuses anything else, so this grants "the tail of a daemon this
        // project ships", not `journalctl`. What a phone in the room learns is what that phone
        // could already learn by watching the robot fail, in words it can put in a support ticket.
        //
        // The one thing worth naming as a cost: a journal line can carry more than a status. Ours
        // are reviewed for that where it matters — `net.connect`'s passphrase is redacted by a
        // hand-written `Debug` with a test pinning it (`proto::NetConnectParams`) — and this
        // routing is the second reason that redaction is load-bearing rather than tidy.
        SystemLogs(_) => true,

        // Rebooting is drastic but recoverable, and it is what an app offers when a robot is
        // confused — the alternative being "unplug it", which for a walking robot is worse.
        // Unlike `resetToGolden` it discards nothing.
        SystemReboot => true,

        // ── the gamepad ─────────────────────────────────────────────────────
        //
        // Pairing a controller from the phone, which is where it belongs: whoever is holding the
        // robot is holding the pad, and the alternative is an ssh session. The same physical-presence
        // argument §4.2 makes for `net.connect` covers it — a pad has to be in the room, in pairing
        // mode, in a fifteen-second window — and it is `configd` that does the work either way.
        //
        // `pad.pair` is the more consequential of the two, because a bonded pad can enable the
        // policy afterwards. That is deliberate: it is the same authority as standing next to the
        // robot with a controller, and the PIN gate is what stands in front of it.
        PadStatus => true,
        PadPair(_) => true,
        PadForget(_) => true,

        // ── refused ─────────────────────────────────────────────────────────

        // The pairing PIN, and the one refusal in this file that is load-bearing rather than
        // conservative: a PIN readable by an unpaired peer authorises nothing at all. `btd`
        // reads it over the unix socket to answer BlueZ's passkey request, and BLE never can.
        SystemPairingPin | SystemSetPairingPin(_) => false,

        // Pinning, and it stays refused while `Select` above it does not. The difference is what
        // the mistake looks like afterwards: a wrong `select` is one release away from being
        // undone and the robot says which release it is on, whereas a robot pinned by a mistap
        // refuses every later update and reports itself as up to date. That is the one failure
        // here that looks exactly like correct behaviour, and it needs `robotctl` and a person
        // who meant it.
        Pin(_) => false,

        // Factory reset in all but name: back to the golden image, discarding every release
        // since. Never over a radio — and note that `Rollback` and `Select` being routed does
        // not weaken this, because neither discards anything.
        ResetToGolden(_) => false,

        // `updaterd`'s private questions to `robotd` — may I restart the control loop, which
        // model API is this, is a telepresence session live. Internal plumbing of the update
        // decision, of no use to a client and misleading if exposed: a phone reading
        // `safeToRestart` would learn nothing it could act on.
        RobotSafeToRestart | RobotModelApi | RobotRemoteSessionActive => false,

        // Motor bus maintenance belongs to its local robotd connection.
        RobotMaintenance(_) | RobotEmergencyRelax => false,

        // Teleop. **Never over BLE**, which is what §4.1 means by a subset: BLE is too slow and
        // too constrained for the full surface, and teleop belongs on WebRTC's datachannel.
        // A 20-byte notification budget and a link that does
        // not exist for the first ~73s of a boot is not a control transport. The body pose and
        // the mouth ride with it: all of these are a stream of small updates, and the argument is
        // about the stream, not about any one of them.
        RobotHumanControl(_) | RobotMove(_) | RobotPrecisionMove(_) | RobotHead(_) | RobotHeadTracking(_) | RobotMotionTracking(_) | RobotCoordinatedTracking(_) | RobotMaze(_) | RobotLook(_) | RobotPose(_) | RobotMouth(_)
        | RobotAccessoryTrigger(_)
        | RobotObstacleStatus | RobotSetObstacle(_)
        | RobotLowVoltageStatus | RobotSetLowVoltage(_) | RobotLowVoltageRearm => false,

        // **Not teleop either, and it sat in that group for the same reason a skill did**: it was
        // next to them in a match arm rather than because anybody argued it belonged. It is one
        // request — start driving, or stop — not fifty a second, so the notification budget and
        // the latency argument above do not reach it.
        //
        // What makes it necessary is what `robot.init` alone turned out not to do. Standing a
        // robot up is the *first* of the two things the gamepad's Start button does; the second
        // is this, and without it the app could stand a robot up and then be told by every skill
        // it asked for that the policy is not driving — press Start on the pad. Half a path is
        // worse than none, because it looks like the whole one until it stops.
        //
        // `toggle` rather than a state this side chose: `padd` keeps no belief about whether the
        // policy is driving, for the reason its own comment gives — a local on/off drifts from
        // the robot's the moment anything else moves it, and a stale belief turns the button into
        // one that does nothing every other press. The robot owns the state and names the one it
        // ended in, so a client shows the answer instead of predicting it.
        RobotEnable(_) => true,

        // **A skill is not teleop**, and it sat in that group for longer than it deserved. It is
        // one request — "do the bow" — not fifty a second, so the notification budget and the
        // latency argument above simply do not reach it. Nor does it need a control link at all:
        // the deadman zeroes the twist by itself, so a robot with nothing driving it stands still
        // and bows.
        //
        // What the refusals in this file mostly turn on is who is *watching*, and BLE answers
        // that better than anything else here does — the radio reaches about ten metres, so a
        // phone that can send this is in the room with the robot by construction. It is also the
        // authenticated transport: the characteristic takes `encrypt_authenticated_write` and the
        // bond is PIN-checked, which WebRTC has no equivalent of (`remote-webrtc.md` §4).
        //
        // Which skills a robot has is config, so a client asks `robot.policies` rather than
        // assuming a list; an unknown name is refused with the names it does know.
        RobotDo(_) => true,

        // Harmless and rather charming from a phone — but it rides the same refusal as the
        // rest of robot.* until the app path exists to want it: opening one call to the
        // radio ahead of a client that can use it buys nothing and widens the surface.
        //
        // The theremin sits here rather than with motor control even though it moves the
        // mouth, because what it is is a sound: the mouth is following the note. Same
        // refusal either way, and the same reason to lift it — an app that can play the duck.
        RobotSound(_) | RobotSoundClip(_) | RobotTheremin(_) | RobotChorale(_) => false,

        // The chorale's own namespace is between `btd` and `robotd` — it is how this daemon is told
        // what to advertise and how it reports what it heard. Not a client surface at all, so a
        // phone asking for it is asking for something that does not exist for it.
        ChoraleSubscribe | ChoraleBeaconSet(_) | ChoraleHeard(_) => false,

        // Powering the machine off from a phone in the room is `system.reboot` without the
        // coming back. The sit-then-power-off flow wants whoever asked to be watching the
        // robot, and that is `robotctl` or the pad's long-press, deliberately.
        RobotShutdown => false,

        // Only a stick-mapping hint for local clients like `padd`. An app gets the same answer
        // through `system.info` territory when it ever needs one; no reason to open another read
        // to the radio today.
        RobotMode => false,

        // Switching modes means the robot goes home, loads other policies and drives differently
        // — and the reason to switch is that somebody just put wheels on it. That is a decision
        // made in the room, holding the pad, the same place `robot.shutdown` is refused for.
        RobotSetMode(_) => false,

        // Loading a policy is `robot.setMode` with a wider blast radius: it puts an arbitrary
        // `.onnx` in charge of fifteen servos, and that file can come from a stranger on the Hub.
        // Everything that makes it survivable — the shape gate, the clamps, the fall reflex — is
        // unchanged whoever asked, so this was never about danger; it was that trying a gait
        // means watching the robot try it, and there was no client that did.
        //
        // Both halves have since turned. There is a client, and BLE is the transport that best
        // meets the watching condition: ten metres of radio range means whoever tapped it is
        // looking at the robot, and the bond is PIN-authenticated. A load that fails keeps the
        // controller that was running, so the failure mode is "nothing happened", not "gaitless
        // robot on the floor".
        //
        // **This method persists.** `robotd` writes the slot key into `robotd.toml` before it
        // queues the swap, so a gait chosen from a phone is the gait the robot boots into. It
        // used to be `robotctl`'s half alone, which made the command and the method the same
        // words with different durability depending on who asked.
        //
        // So this is a durable remote change, like `pad.bind` below, and belongs on the
        // PIN-bonded transport for that reason as much as for the watching one. Undoing it is
        // `robot.loadPolicy` with no path, which is reachable from the same phone.
        RobotLoadPolicy(_) => true,
        RobotManagedModels | RobotModelUpdateStatus(_) => true,
        RobotApplyModel(_) => false,

        // Which button runs which skill, and changing one. **This is the transport those exist
        // for**: `robotctl pad bind` is for whoever is holding the robot, and a phone that can
        // already ask for a skill is the obvious place to decide which button asks for it.
        //
        // `pad.bind` writes the config file, and has to: `padd` re-reads `[pad]` every second,
        // so a binding held in memory would be reverted before the caller let go of the phone.
        //
        // A name is checked against the skills this robot has before anything is written, so the
        // failure mode is a refusal naming them rather than a button that does nothing.
        PadBindings | PadBind(_) => true,

        // The skill table: what this robot can be asked to do, and adding to it. The last thing
        // in the policy path that only a terminal on the robot could reach — `[[policy.skill]]`
        // is a repeating table, so `robotctl policy add` wrote it directly and there was nothing
        // to route.
        //
        // `robotd` writes the file and reloads itself, so one call is the whole operation. A
        // client that had to remember a second one and forgot would leave a robot whose config
        // and behaviour disagree until the next restart.
        RobotSkills | RobotSetSkill(_) | RobotRemoveSkill(_) => true,

        // What each slot is running, and which skills this robot has. Read-only, and the read a
        // client makes before it can offer either of the two above: there is no compiled-in list
        // of skills to assume any more, so this is how a phone knows there is a bow to ask for.
        RobotPolicies => true,

        // Static geometry, read-only; the same class of read as the one above.
        RobotModel => true,

        // Re-reading the slots after something else edited the config. Same blast radius as
        // `robot.loadPolicy` and the same answer, and a client that can load wants this for the
        // case where the file changed underneath it.
        RobotReloadPolicies => true,

        // Is there a newer official policy set, and what else is on the Hub. Both reach the
        // network, and neither changes anything on the robot — the same kind of question as
        // `update.check`, which has been routed here since the update path was driven from a
        // phone.
        //
        // They were refused on the grounds that answering "yes, there is a newer gait" for a
        // client that could not then install one is an odd thing to offer. That was fair while it
        // was true and stops being an argument the moment `policy.install` is routed, which is
        // the next arm.
        PolicyCheck | PolicySearch(_) => true,

        // And installing one. This is the most obviously *appealing* thing in this file: a
        // stranger's gait, from a phone, onto the robot in front of you.
        //
        // What makes it survivable is unchanged and was always the point — the manifest gate
        // before the download, the shape gate at load, the joint clamps, the fall reflex — so
        // this was never about danger. It was about who is *watching*, and BLE answers that
        // better than anything else here: ten metres of radio range means whoever tapped it is
        // looking at the robot, and the bond is PIN-checked.
        //
        // `policy.install` is `is_mutating`, so `updaterd` authorises it against the peer's
        // uid — which is `btd`'s, not the phone's, exactly as it already is for `update.apply`.
        // The transport is the gate here, not the credential.
        PolicyFetch(_) | PolicyInstall(_) => true,

        // The detector's set, by the same argument: a read that reaches the network, and an
        // install whoever tapped it is standing next to.
        DetectorCheck | DetectorInstall(_) => true,

        // ── the account, which BLE is the right transport for ────────────────
        //
        // Signing the robot in to a Hugging Face account is what makes it reachable from outside
        // the LAN at all, and this is the transport that should carry it — for the reason
        // `remote-webrtc.md` §4 gives about BLE generally: it is PIN-bonded, and ten metres of
        // radio range means whoever tapped the button is in the room with the robot.
        //
        // It also happens to be the *only* transport that can do it on a robot fresh out of a
        // box. A duck with no wifi has no console to open and no LAN to open it from, and BLE is
        // already how such a robot is given a network — so a wizard that joins the wifi and then
        // signs the robot in is one flow on one transport, which is what the mini's app does.
        //
        // The code the flow produces has to be *read by a person*, so the reply carrying it is
        // the point: this call sends about eighty bytes back, well inside what framing chunks,
        // and needs no link at all while the user is off approving it — see
        // `duck_ipc_proto::API_VERSION`'s v23 note on why `login` answers with a code and not
        // with a token. That is what makes an iPhone dropping the GATT link mid-flow a
        // non-event rather than a lost login.
        AccountLogin(_) | AccountStatus | AccountLogout => true,

        // **Standing the robot up, which is how anything else here starts.**
        //
        // Refused until the phone app was used, on the grounds that standing a robot up moves
        // every joint at once and wants the person doing it to be looking at the robot rather
        // than at a screen. The second half of that is the argument this file makes *for*
        // routing things — ten metres of radio range means whoever tapped it is looking at the
        // robot — and it is what lets `robot.do`, `robot.loadPolicy` and `policy.install`
        // through. It was never a reason to refuse this one.
        //
        // What made it worth changing is what the refusal cost: a robot that has not been
        // started ignores every other call this transport carries, so the app could show a
        // robot's health, its wifi and its gaits and not make it move — and the way out was to
        // go and find the gamepad. That is the opposite of what the app is for.
        //
        // `robotd` already publishes what a client needs to choose correctly: `homed` and
        // `sitting` on `robot.policies`, added in API v30 and v31 for exactly this — a duck on
        // its feet stands with `init`, a duck in its seat is held there by the `sit_toggle`
        // latch and `init` argues with it rather than winning. A client that has those does not
        // have to guess.
        RobotInit => true,

        // **The way back from a latched servo error, which is where a phone is the only tool.**
        //
        // It was refused beside `relax`, on the reading that cycling the servo bus while the
        // robot is standing is the same fall by another route. That is true of the instruction
        // and wrong about the situation: a servo that has latched overload, overheating or
        // electrical shock is holding nothing already, and nothing else clears it but pulling
        // the battery. So the call an owner needs is refused exactly when the robot is on the
        // floor with a dead joint and there is nothing to fall.
        //
        // What made it worth changing is that no other way back reaches a phone. Standing up is
        // `init`, routed above — and `init` on a robot with a latched servo does nothing but
        // fail again, which is the state the app can now see (`robot.health` names the joint)
        // and could not leave. The rest of the path is already here: torque goes off on every
        // joint first, the servos come back limp with their gains restored on the next write,
        // and `init` brings it up from there.
        //
        // The hazard is real for the robot that *is* standing, and it is the one the app is
        // asked to state: this drops the joints, so have the robot down or hold it. That is the
        // same claim `robot.do` and `robot.loadPolicy` are routed on — ten metres of radio range
        // means whoever tapped it is looking at the robot.
        RobotRebootMotors(_) => true,

        // A rest ends where a servo reboot does — limp — and gets there more gently: sat down
        // and eased into the rest pose first when the robot is driving. Permitted on
        // `rebootMotors`' argument; it is `relax`'s unannounced drop it avoids, not its outcome.
        RobotRest => true,

        // **`relax` stays refused, and the asymmetry is the point.** Standing up is controlled:
        // the joints go where they are told. Relaxing is a robot that was holding itself up and
        // now is not, which on a phone is a button whose failure mode is the floor, and its
        // whole purpose is to produce that fall — unlike the reboot above, which exists to
        // recover a robot that has already stopped holding itself. `init` has no such mode, and
        // a refusal that covered all three was treating "moves the joints" as the hazard when
        // the hazard is "stops holding them for nothing in return".
        RobotRelax => false,

        // `robot.stop` deserves its own line, because refusing it looks wrong. An emergency stop
        // in the app is exactly what someone reaches for, and §6 does say local should preempt
        // remote — but a stop button that works over an unbonded, high-latency, sometimes-absent
        // radio is worse than no button, because it *looks* like an e-stop and is not one. The
        // deadman in `robotd` already stops the robot when intents stop arriving, which is the
        // mechanism that does not depend on a phone being in range. A real e-stop is physical.
        // Reconsider deliberately if the app ever needs it, with that caveat stated in the UI.
        RobotStop => false,

        // High-rate telemetry. `robot.subscribe` streams state at up to the control rate; over
        // BLE that is a firehose into a 20-byte pipe, and a client would get a decimated,
        // unpredictably-lagged view it could not reason about. `robot.health` is the question an
        // app actually has.
        RobotSubscribe(_) => false,

        // The same objection as `robot.subscribe`, only more so: this is every evdev event the pad
        // sends, over a hundred reports a second, and it exists to *measure the cadence of its own
        // delivery*. Carried over BLE the measurement would be of the phone's link rather than the
        // pad's, which is worse than refusing — it would be a number that looks like an answer.
        //
        // It is also not `btd`'s to forward: `padd` is deliberately not one of the sockets `btd`
        // holds, which `Upstream`'s conversion now enforces rather than merely documents.
        PadInput => false,

        // Depth frames, and the same two objections as the pad tap. A 64-zone frame
        // fifteen times a second is a firehose into a 20-byte pipe; and it is served by
        // `tofd`, which is not one of the sockets `btd` holds. When a phone has a
        // reason to see what the robot sees, it will be through `mediad`'s video path,
        // where depth belongs next to the frame it annotates.
        TofStream => false,
        // Same as the ToF: the head IMU is tofd's, reached over mediad's video path, not BLE.
        HeadImuStream => false,
    }
}

/// Where this call goes and on which connection, or `None` if BLE may not make it — or if no
/// service answers it, which for `system.authenticate` is the same answer with a different reason.
/// [`route_for`] tells those two apart.
pub fn destination_for(call: &proto::Call) -> Option<(Upstream, Lane)> {
    if !permits(call) {
        return None;
    }
    let (service, lane) = call.destination()?;
    Some((Upstream::try_from(service).ok()?, lane))
}

/// The service that answers a call, ignoring which connection carries it.
///
/// The permission question on its own, which is what most callers and every test about the
/// security boundary are asking.
pub fn upstream_for(call: &proto::Call) -> Option<Upstream> {
    destination_for(call).map(|(upstream, _)| upstream)
}

/// The full routing decision, including the one call the transport answers itself.
pub fn route_for(call: &proto::Call) -> Route {
    match call {
        proto::Call::SystemAuthenticate(_) => Route::Local,
        other => match destination_for(other) {
            Some((upstream, lane)) => Route::To(upstream, lane),
            None => Route::Refused,
        },
    }
}

/// The JSON-RPC error to answer a refused call with.
///
/// [`proto::code::PERMISSION_DENIED`] rather than `METHOD_NOT_FOUND`, because the two mean
/// different things to whoever is holding the phone: this method exists and this transport
/// may not use it — "try `robotctl`", not "upgrade your app".
pub fn refusal(call: &proto::Call) -> proto::Error {
    proto::Error::new(
        proto::code::PERMISSION_DENIED,
        format!(
            "{} is not available over Bluetooth; use robotctl on the robot",
            call.method()
        ),
    )
}
