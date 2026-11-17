//! Which Hugging Face account this robot belongs to — the duck's half of it.
//!
//! The flow itself is [`hf_robot_account`]: an OAuth device grant against Hugging Face, a
//! credential on disk, and a loop that renews it. It was written here and lives there now,
//! because a robot signing in to an account has nothing to do with updating one, and because
//! every other robot wants exactly the same thing.
//!
//! What stays is everything that is a fact about *this* robot rather than about the flow:
//!
//! - **[`DEFAULT_PATH`] and [`TOKEN_GROUP`]**, which are this board's filesystem and this board's
//!   user database.
//! - **The mapping onto [`proto`]**, below. The crate's types happen to serialise identically
//!   today, so `Response::ok` could be handed one directly and the wire would not change — and
//!   that is exactly the shortcut not to take. `API_VERSION` versions these structs; a field
//!   renamed in a crate we do not release must be a compile error here, not a silently different
//!   answer to a client that has been told the version did not move.
//! - **The error mapping**, in `impl From<hf_robot_account::Error> for crate::Error`, because
//!   which JSON-RPC code a refusal deserves is a question about this protocol.
//!
//! An account is what makes a robot reachable from *outside* the LAN: the relay proves the robot
//! belongs to an account, the rendezvous service shows a client only its own robots, and the pair
//! of those is the authorisation a bridged session arrives with. A duck on a LAN needs none of it.

use hf_robot_account::{Config, FileStore, Identity, LoginCode, Status};

use crate::proto;

pub use hf_robot_account::{Account, maintain};

/// Where the credential lives.
///
/// **Not in `robotd.toml`.** Every mechanism that exists for that file is wrong for a secret:
/// `robotctl configure --list` prints what a robot changes, the config editor shows the whole
/// file, and "what was changed on this robot" is a report we generate. A bearer token would be in
/// all three.
pub const DEFAULT_PATH: &str = "/etc/robot/hf-token";

/// The group that may read the token file.
///
/// `mediad` runs as `User=mediad` with `SupplementaryGroups=robot`, and it is the process that
/// needs the token — it is the one holding the relay. `updaterd` runs as root and writes it. So
/// the file is `root:robot` and `0640`: readable by the daemons that belong to this robot, and by
/// nothing else with a login on the board.
pub const TOKEN_GROUP: &str = "robot";

/// The account this robot belongs to, at [`DEFAULT_PATH`].
pub fn account() -> Account {
    Account::new(store(DEFAULT_PATH), Config::from_env())
}

/// As [`account`], with the credential somewhere other than a robot's.
///
/// **For a duck in MuJoCo, and it is what makes one signable-in at all.** The device flow, the
/// refresh and the file `mediad` reads are the same code on a laptop as on a board; the one thing
/// a laptop cannot do is write `/etc/robot/hf-token`. So the path moves and nothing else does — a
/// simulated duck signs in to a Hugging Face account by typing a code exactly as a real one does,
/// and appears in the same listing afterwards. `docs/robot/simulation.md` has the walkthrough.
///
/// Distinct from [`account_for_test`], which also redirects the *endpoint*: this one still talks to
/// Hugging Face, because a simulated duck's account is a real account.
pub fn account_at(path: impl Into<std::path::PathBuf>) -> Account {
    Account::new(store(path), Config::from_env())
}

/// As [`account`], against a token file and a Hugging Face of your choosing.
///
/// Only for tests, and it is not optional there: the real path is a robot's real credential, and
/// a test that ran against it would read — or on a developer's machine try to write — that.
#[doc(hidden)]
pub fn account_for_test(path: impl Into<std::path::PathBuf>, endpoint: String) -> Account {
    Account::new(store(path), Config::at(endpoint))
}

fn store(path: impl Into<std::path::PathBuf>) -> FileStore {
    FileStore::at(path).readable_by_group(TOKEN_GROUP)
}

// ── onto the wire ────────────────────────────────────────────────────────────

/// The answer to `account.login`.
pub fn login_result(code: LoginCode) -> proto::AccountLoginResult {
    proto::AccountLoginResult {
        user_code: code.user_code,
        verification_uri: code.verification_uri,
        verification_uri_complete: code.verification_uri_complete,
        expires_in: code.expires_in,
        interval: code.interval,
    }
}

/// The answer to `account.status`.
pub fn status_result(status: Status) -> proto::AccountStatusResult {
    proto::AccountStatusResult {
        account: status.account.map(identity),
        login: status.login.map(login_result),
        last_error: status.last_error,
    }
}

fn identity(who: Identity) -> proto::Account {
    proto::Account {
        username: who.username,
        token_expires_in: who.token_expires_in,
        refreshable: who.refreshable,
    }
}

/// The answer to `account.logout`.
pub fn logout_result(was: Option<String>) -> proto::AccountLogoutResult {
    proto::AccountLogoutResult { was }
}
