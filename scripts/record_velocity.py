"""固定录制入口；可显式选择参考IMU重力；--name同名覆盖。"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time


def recording_filename(value):
    if not re.fullmatch(r"[\w][\w.-]*\.jsonl", value):
        raise argparse.ArgumentTypeError("名称须为单个.jsonl文件名，可用文字、数字、下划线、点或短横线")
    return value


def diagnostic_ids(value):
    try:
        ids = [int(part) for part in value.split(',')]
    except ValueError as error:
        raise argparse.ArgumentTypeError("诊断ID须为逗号分隔的整数") from error
    available = set(range(10, 15)) | set(range(20, 25)) | set(range(30, 35))
    if not ids or len(set(ids)) != len(ids) or not set(ids) <= available:
        raise argparse.ArgumentTypeError("诊断ID须不重复，且属于10–14、20–24、30–34；不包含IMU")
    return ids


def selected_policy_slots(runtime_args):
    """读取显式启用的槽位，使模型路径写入实际运行的槽。"""
    selection = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    selection.add_argument("--enable-policies", action="append", default=[])
    options, _ = selection.parse_known_args(runtime_args)
    return {slot for slots in options.enable_policies for slot in slots.split(",")}


def velocity_slot_requested(runtime_args):
    """velocity槽固定加载walk配置同目录的velocity.onnx。"""
    return "velocity" in selected_policy_slots(runtime_args)


def recording_arguments(runtime_args, deployment_like=False):
    """验证IMU来源，为逐帧录制启用主机控制循环计时。"""
    selection = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    selection.add_argument("--imu-source", action="append", default=[],
                          choices=("i2c", "id200", "id200-timing", "id201", "id200-host"))
    selection.add_argument("--reference-imu-source", action="append", default=[],
                          choices=("i2c", "id200", "id200-timing", "id201"))
    selection.add_argument("--measure-servo-read-time", action="store_true")
    selection.add_argument("--policy-gravity-from-reference", action="count", default=0)
    selection.add_argument("--imu-filter", action="append", default=[])
    selection.add_argument("--fake", action="store_true")
    selection.add_argument("--sim")
    options, _ = selection.parse_known_args(runtime_args)
    if len(options.imu_source) > 1:
        raise ValueError("只允许指定一次--imu-source")
    if len(options.reference_imu_source) > 1:
        raise ValueError("只允许指定一次--reference-imu-source")
    if options.policy_gravity_from_reference > 1:
        raise ValueError("只允许指定一次--policy-gravity-from-reference")
    if not options.imu_source:
        raise ValueError("请用--imu-source明确指定主IMU")
    reference = options.reference_imu_source[0] if options.reference_imu_source else None
    requested_source = options.imu_source[0]
    gravity_from_reference = bool(options.policy_gravity_from_reference)
    identity = lambda source: "id200" if source in ("id200", "id200-timing", "id200-host") else source
    policy_source = lambda source: "id200" if source == "id200-timing" else source
    if reference is not None and identity(reference) == identity(requested_source):
        raise ValueError("主IMU与副IMU必须是不同的传感器")
    if deployment_like and reference is not None:
        raise ValueError("--reference-imu-source需要正常逐帧录制")
    if gravity_from_reference:
        if deployment_like:
            raise ValueError("--policy-gravity-from-reference需要正常逐帧录制")
        if options.fake or options.sim is not None:
            raise ValueError("--policy-gravity-from-reference要求真实总线")
        if (requested_source, reference) not in (
                ("id200", "id201"), ("id200-timing", "id201"),
                ("id201", "id200"), ("id201", "id200-timing")):
            raise ValueError("--policy-gravity-from-reference要求显式选择不同的ID200和ID201")
        if options.imu_filter != ["false"]:
            raise ValueError("--policy-gravity-from-reference要求明确且仅一次指定--imu-filter false")
        if os.environ.get("ROBOT_FREEZE_POLICY_IMU") == "1":
            raise ValueError("--policy-gravity-from-reference不能与ROBOT_FREEZE_POLICY_IMU=1合用")
    forwarded = list(runtime_args)
    if not deployment_like and not options.measure_servo_read_time:
        forwarded.append("--measure-servo-read-time")
    return forwarded, dict(requested_source=requested_source,
                           effective_source=requested_source,
                           policy_gravity_from_reference=gravity_from_reference,
                           policy_gyro_source=policy_source(requested_source),
                           policy_gravity_source=policy_source(reference if gravity_from_reference
                                                               else requested_source),
                           reference_source=reference,
                           reference_mcu_timing=reference in ("id200", "id200-timing", "id201"),
                           mcu_timing=requested_source in ("id200-timing", "id201"),
                           control_loop_timing=(not deployment_like or
                                                options.measure_servo_read_time))


def select_policy_for_run(policy, policy_sha256, run, runtime_args):
    if not velocity_slot_requested(runtime_args):
        return policy.resolve(), None
    # The velocity slot rewrites [policy].walk to a sibling velocity.onnx.
    # Freeze this run's model there without modifying the global alias.
    selected = run / "velocity.onnx"
    shutil.copyfile(policy, selected)
    if hashlib.sha256(selected.read_bytes()).hexdigest() != policy_sha256:
        raise ValueError("本次velocity模型副本校验失败，未访问硬件")
    return selected, dict(path=str(selected), sha256=policy_sha256)


def prepare_run(root, name):
    """命名记录使用独立同名目录；覆盖范围限定在该目录。"""
    if root.is_symlink():
        raise ValueError("records目录不能是符号链接")
    root.mkdir(exist_ok=True)
    if name is None:
        number = max([int(p.name.split("_")[0]) for p in root.iterdir()
                      if p.is_dir() and p.name.split("_")[0].isdigit()] + [0]) + 1
        run = root / f"{number:03d}_velocity"
        filename = "record.jsonl"
    else:
        filename = recording_filename(name)
        run = root / Path(filename).stem
        if run.is_symlink():
            raise ValueError("同名记录目录不能是符号链接")
        if run.exists():
            if not run.is_dir():
                raise ValueError("同名路径不是记录目录")
            print(f"同名覆盖：{run}（该目录内旧记录及配套文件将被替换）", flush=True)
            shutil.rmtree(run)
    run.mkdir(exist_ok=False)
    return run, run / filename


def run_control_process(command, environment):
    """转发终端停止信号，等待控制程序收尾后才允许再次使用串口。"""
    process = None
    received = []
    pending = []

    def forward(signum, _frame):
        received.append(signum)
        if process is None:
            pending.append(signum)
        elif process.poll() is None:
            try:
                os.killpg(process.pid, signum)
            except ProcessLookupError:
                # 进程可能在poll与killpg之间正常退出；wait仍负责回收。
                pass

    previous = {sig: signal.getsignal(sig) for sig in
                (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    started = time.monotonic_ns()
    try:
        for sig in previous:
            signal.signal(sig, forward)
        process = subprocess.Popen(command, env=environment, start_new_session=True)
        # 极短的创建窗口内收到的信号也必须交给子进程。
        for signum in tuple(pending):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signum)
                except ProcessLookupError:
                    pass
        code = process.wait()
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return dict(returncode=code, signals_received=received,
                started_t_ns=started, exited_t_ns=time.monotonic_ns())


def compare_diagnostic_endpoints(before_path, after_path, ids):
    """计算累计量差值；历史最大值不冒充本次测试最大值。"""
    def load(path):
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        samples = {}
        for row in rows:
            if row['type'] == 'servo_diagnostics_baseline':
                if row['id'] in samples:
                    raise ValueError('端点记录包含重复舵机ID')
                samples[row['id']] = row
        return samples, any(row['type'] == 'completion' for row in rows)

    before, before_complete = load(before_path)
    after, after_complete = load(after_path)
    keys = ('isr_max_us', 'isr_overrun_count', 'sensor_read_fail_count')
    samples = []
    maximum = (1 << 32) - 1
    for device in ids:
        a, b = before.get(device), after.get(device)
        if a is None or b is None:
            samples.append(dict(id=device, status='missing_snapshot',
                                before_present=a is not None, after_present=b is not None))
            continue
        for row in (a, b):
            if any(type(row[key]) is not int or not 0 <= row[key] <= maximum for key in keys):
                raise ValueError('诊断累计值不符合u32范围')
        reset = any(b[key] < a[key] for key in keys)
        saturated = [key for key in keys if a[key] == maximum or b[key] == maximum]
        delta = {key: None if reset or key in saturated else b[key] - a[key]
                 for key in keys[1:]}
        samples.append(dict(id=device, status='reset_or_discontinuity' if reset else 'observed',
            before={key: a[key] for key in keys}, after={key: b[key] for key in keys}, delta=delta,
            counter_reset_or_discontinuity=reset, saturated_fields=saturated,
            test_maximum_us_if_new_record=(b['isr_max_us'] if not reset
                and 'isr_max_us' not in saturated and b['isr_max_us'] > a['isr_max_us'] else None),
            support_confirmed=False,
            interval_start_t_ns=a['read_start_t_ns'], interval_end_t_ns=b['read_end_t_ns']))
    return dict(before_file=str(before_path), after_file=str(after_path),
        snapshots_complete=before_complete and after_complete
            and all(s['status'] != 'missing_snapshot' for s in samples),
        deltas_require_no_unobserved_reset=True, servos=samples,
        limits=['计数在RAM中累计，断电/复位清零；端点未下降不能排除中途复位后再次累加。',
                '相同历史最大值不能给出本次测试最大耗时；只有超过测试前历史纪录才填本次新最大值。',
                '端点覆盖整次进程及退出收尾，不能定位到某次摔倒或单个策略帧。',
                '全零不证明旧固件支持诊断；饱和、缺项或失败不当作零新增。'])


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--robotd", type=Path,
                        help="可选；默认当前源码目录target/release/robotd")
    parser.add_argument("--robotd-sha256",
                        help="可选的robotd预期SHA256；总会记录实际文件SHA256")
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--policy-sha256",
                        help="可选的策略预期SHA256；总会记录实际文件SHA256")
    parser.add_argument("--params", type=Path, required=True,
                        help="原始robotd配置；将录制模型写入本次配置副本的对应策略槽")
    parser.add_argument("--port",
                        help="robotd和寄存器诊断共用的串口；诊断默认使用/dev/ttyS3")
    parser.add_argument("--imu-source", action="append", required=True,
                        choices=("i2c", "id200", "id200-timing", "id201", "id200-host"),
                        help="主IMU来源，只能指定一次。ID200的MCU时序需显式选择id200-timing及配套固件。")
    parser.add_argument("--reference-imu-source", action="append", default=[],
                        choices=("i2c", "id200", "id200-timing", "id201"),
                        help="可选副IMU：默认仅记录；不传则不探测、不读取副IMU")
    parser.add_argument("--imu-last", action="store_true",
                        help="让IMU在全部舵机后回复，用于减少本轮IMU数据年龄的对照")
    parser.add_argument("--with-gamepad", action="store_true",
                        help="原样传给robotd，启动同目录padd；速度上限由用户参数决定")
    parser.add_argument("--motion-only", action="store_true",
                        help="关闭相机、ToF和网页自动启动；保留策略、录制及可选手柄")
    parser.add_argument("--name", type=recording_filename,
                        help="例如i2c400k.jsonl；保存至records/i2c400k/，同名目录整次覆盖")
    parser.add_argument("--servo-diagnostics-ids", type=diagnostic_ids,
                        help="前后读取新固件0x80–0x8B，例如11,12,21；全零不能证明固件支持")
    parser.add_argument("--servo-diagnostics-mode", choices=("endpoints", "periodic"),
                        default="endpoints", help="默认只在测试前后读；periodic另启用运行中的轮询")
    parser.add_argument("--deployment-like", action="store_true",
                        help="跳过寄存器快照和逐帧采集；不传--record、--log-file或舵机周期诊断")
    args, runtime_args = parser.parse_known_args()
    if args.motion_only and any(value.split("=", 1)[0] == "--with-camera" for value in runtime_args):
        parser.error("--motion-only不能与--with-camera同时使用")
    imu_args = [item for source in args.imu_source for item in ("--imu-source", source)]
    imu_args += [item for source in args.reference_imu_source
                 for item in ("--reference-imu-source", source)]
    runtime_args = imu_args + runtime_args
    effective_runtime_args, imu_recording = recording_arguments(runtime_args, args.deployment_like)
    if any(value.split("=", 1)[0] in ("--record", "--log-file") for value in runtime_args):
        parser.error("采集输出由--name管理，请勿同时传--record或--log-file")
    if (not args.deployment_like and args.servo_diagnostics_mode == 'periodic'
            and not args.servo_diagnostics_ids):
        parser.error('periodic模式需要--servo-diagnostics-ids')
    if os.geteuid() != 0:
        raise PermissionError("请使用sudo启动采集，以访问串口和所选IPC socket")
    base = Path(__file__).resolve().parent
    binary = (args.robotd or base.parent / "target/release/robotd").resolve(strict=True)
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise ValueError("所选robotd不是可执行文件，未启动、未覆盖记录")
    binary_sha256 = hashlib.sha256(binary.read_bytes()).hexdigest()
    if args.robotd_sha256 is not None and binary_sha256 != args.robotd_sha256:
        raise ValueError("所选robotd与指定SHA256不一致，未启动、未覆盖记录")
    if imu_recording['policy_gravity_from_reference'] or args.motion_only:
        help_result = subprocess.run([str(binary), '--help'], capture_output=True, text=True, check=True)
        if imu_recording['policy_gravity_from_reference'] and '--policy-gravity-from-reference' not in help_result.stdout:
            raise ValueError("所选robotd不支持参考IMU重力，未访问硬件、未覆盖记录")
        if args.motion_only and '--motion-only' not in help_result.stdout:
            raise ValueError("所选robotd不支持--motion-only，未访问硬件、未覆盖记录")
    helper = base / "imu-live.py"
    config_source = args.params
    policy_sha256 = hashlib.sha256(args.policy.read_bytes()).hexdigest()
    if args.policy_sha256 is not None and policy_sha256 != args.policy_sha256:
        raise ValueError("所选策略与指定比较模型不一致，未启动")
    gamepad = binary.with_name("padd")
    if args.with_gamepad and (not gamepad.is_file() or not os.access(gamepad, os.X_OK)):
        raise FileNotFoundError("--with-gamepad要求所选robotd同目录有可执行padd，未启动、未覆盖记录")
    source = config_source.read_text()
    section = re.search(r"(?ms)^\[policy\]\s*\n(.*?)(?=^\[|\Z)", source)
    if section is None:
        raise ValueError("缺少policy配置节")
    slots = selected_policy_slots(runtime_args)
    if slots in ({"sitstand"}, {"sitstand", "stand"}):
        policy_slot = "sitstand"
    else:
        policy_slot = "stand" if slots == {"stand"} else "walk"
    policy_pattern = rf"(?m)^{policy_slot}\s*=.*$"
    if len(re.findall(policy_pattern, section.group(1))) != 1:
        raise ValueError(f"要求且只允许一个{policy_slot}策略入口")
    port = args.port or "/dev/ttyS3"
    if not Path(port).is_char_device():
        raise FileNotFoundError(f"无法访问串口 {port}，请用 --port 指定实机串口")
    owner = subprocess.run(["fuser", "-s", port], check=False)
    if owner.returncode != 1:
        raise RuntimeError(f"串口 {port} 被占用或无法确认占用状态，未启动")
    records_root = base.parent.parent / "records"
    if records_root.is_symlink():
        raise ValueError("records目录不能是符号链接")
    records_root.mkdir(exist_ok=True)
    run_root = records_root / "deployment-like" if args.deployment_like else records_root
    run, record_path = prepare_run(run_root, args.name)
    selected_policy, velocity_alias = select_policy_for_run(
        args.policy, policy_sha256, run, runtime_args)
    policy_line = policy_slot + " = " + json.dumps(str(selected_policy))
    policy_text, count = re.subn(policy_pattern, policy_line, section.group(1))
    if count != 1:
        raise ValueError(f"要求且只允许一个{policy_slot}策略入口")
    config = source[:section.start(1)] + policy_text + source[section.end(1):]
    params = run / "params.toml"
    params.write_text(config)
    preflight = None
    if not args.deployment_like:
        preflight = [sys.executable, str(base / "read_registers.py"), "--helper", str(helper),
                     "--port", port, "--output", str(run / "registers-before.jsonl")]
        if args.servo_diagnostics_ids:
            preflight += ["--diagnostics-ids", *map(str, args.servo_diagnostics_ids)]
        subprocess.run(preflight, check=True)
    command = [str(binary), *effective_runtime_args, "--params", str(params)]
    if args.port:
        command += ["--port", args.port]
    environment = os.environ.copy()
    if args.port == "/run/wonderduck-unoq/bus":
        environment.setdefault("WONDERDUCK_UNOQ_FEEDBACK", "1")
        environment.setdefault("WONDERDUCK_MOTION_CPU_DMA_LATENCY_US", "100")
    if not args.deployment_like:
        command += ["--log-file", str(run / "robotd.log"), "--record", str(record_path)]
    if args.imu_last:
        command.append("--imu-last")
    if args.with_gamepad:
        command.append("--with-gamepad")
    if args.motion_only:
        command.append("--motion-only")
    if not args.deployment_like and args.servo_diagnostics_ids and args.servo_diagnostics_mode == 'periodic':
        command += ["--servo-diagnostics-ids", ','.join(map(str, args.servo_diagnostics_ids))]
    receipt = dict(command=command, policy_path=str(args.policy), policy_sha256=policy_sha256,
                   expected_policy_sha256=args.policy_sha256,
                   selected_policy_path=str(selected_policy),
                   selected_policy_slot=policy_slot,
                   velocity_alias=velocity_alias,
                   binary_path=str(binary), binary_sha256=binary_sha256,
                   expected_binary_sha256=args.robotd_sha256,
                   config_source_path=str(config_source),
                   config_source_sha256=hashlib.sha256(source.encode()).hexdigest(),
                   protocol="Register40/28-byte servo broadcast; IMU transport selected by --imu-source", imu_last=args.imu_last,
                   with_gamepad=args.with_gamepad,
                   motion_only=args.motion_only,
                   gamepad_path=str(gamepad) if args.with_gamepad else None,
                   gamepad_sha256=hashlib.sha256(gamepad.read_bytes()).hexdigest() if args.with_gamepad else None,
                   deployment_like=args.deployment_like,
                   servo_diagnostics_ids=None if args.deployment_like else args.servo_diagnostics_ids,
                   skipped_servo_diagnostics_ids=args.servo_diagnostics_ids if args.deployment_like else None,
                   servo_diagnostics_mode=args.servo_diagnostics_mode,
                   servo_diagnostics_scope=("skipped" if args.deployment_like else
                       "before/after snapshots; in-loop queries only in periodic mode"),
                   control_source=("robotd-owned padd" if args.with_gamepad else "external IPC client"),
                   requested_robotd_arguments=runtime_args,
                   imu_recording=imu_recording,
                   environment_inputs={key: environment.get(key) for key in
                       ("ORT_DYLIB_PATH", "ROBOT_FREEZE_POLICY_IMU", "RUST_LOG", "WONDERDUCK_UNOQ_FEEDBACK",
                        "WONDERDUCK_MOTION_CPU_DMA_LATENCY_US")},
                   recording_filename=None if args.deployment_like else record_path.name,
                   overwrite_named_record=args.name is not None)
    (run / "request.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(f"本次运行目录：{run}", flush=True)
    if args.deployment_like:
        print("接近部署模式：跳过前后寄存器读取及逐帧采集；不生成运动JSONL。", flush=True)
    else:
        print(f"本次记录文件：{record_path}", flush=True)
        if imu_recording['mcu_timing']:
            print(f"MCU时序随策略写入同一JSONL（{imu_recording['effective_source']}），"
                  "超时及循环耗时写入robotd.log。",
                  flush=True)
    print("主IMU：" + imu_recording['policy_gyro_source'].upper()
          + ("（角速度用于推理）" if imu_recording['policy_gravity_from_reference']
             else "（角速度和重力用于推理）"), flush=True)
    if imu_recording['reference_source'] is None:
        print("副IMU：未启用（不探测、不读取）", flush=True)
    else:
        print("副IMU：" + imu_recording['reference_source'].upper()
              + ("（重力用于推理，亦记录）" if imu_recording['policy_gravity_from_reference']
                 else "（仅记录，不参与推理）"), flush=True)
    print(f"本次控制程序：{binary}；SHA256：{binary_sha256}", flush=True)
    if args.servo_diagnostics_ids and not args.deployment_like:
        print("舵机诊断ID：" + ','.join(map(str, args.servo_diagnostics_ids))
              + "；模式=" + args.servo_diagnostics_mode + "；退出后自动读取并比较累计值。", flush=True)
    print("启用后先零指令保持10秒；正常时短前进约0.5秒、归零5秒，最多三次。出现失稳按原方法停策略。", flush=True)
    result = run_control_process(command, environment)
    result['after_registers_returncode'] = None
    result['endpoint_comparison_error'] = None
    after_failed = False
    if args.servo_diagnostics_ids and not args.deployment_like:
        print('控制程序已退出，正在读取测试后寄存器；请保持舵机供电。', flush=True)
        postflight = preflight.copy()
        after_path = run / 'registers-after.jsonl'
        postflight[postflight.index('--output') + 1] = str(after_path)
        try:
            post = subprocess.run(postflight, check=False)
            result['after_registers_returncode'] = post.returncode
            after_failed = post.returncode != 0
            comparison = compare_diagnostic_endpoints(run / 'registers-before.jsonl',
                                                      after_path, args.servo_diagnostics_ids)
            (run / 'servo-diagnostics-comparison.json').write_text(
                json.dumps(comparison, ensure_ascii=False, indent=2) + '\n')
            after_failed = after_failed or not comparison['snapshots_complete']
        except (OSError, ValueError, KeyError, KeyboardInterrupt) as error:
            result['endpoint_comparison_error'] = str(error) or '用户取消测试后读取'
            after_failed = True
        if after_failed:
            print('测试后读取或对比未完成，策略原始记录已保留；详见run-result.json。',
                  file=sys.stderr, flush=True)
        else:
            print('测试前后寄存器和累计值对比已保存。', flush=True)
    (run / 'run-result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    code = result['returncode']
    return (128 - code if code < 0 else code) if code != 0 else int(after_failed)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('采集已取消，已生成的记录文件保留。', file=sys.stderr)
        raise SystemExit(130)
