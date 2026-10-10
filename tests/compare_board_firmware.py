"""在临时目录编译真实 GD32C113 协议源文件与SDK，做无硬件双向互操作测试。"""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

HARNESS = r'''
/* 使用板仓已有的硬件替身；不复制或修改板级代码。 */
/* 原测试main隐式return 0；重命名后不再享有C的main特例，且这里不会调用它。 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main reference_test_main
#include "test_protocol.c"
#undef main
#pragma GCC diagnostic pop
#include "uniubi6913_board.h"
int main(void) {
    u6913_frame_t f;
    u6913_board_packet_t packet;
    u6913_board_event_t event;
    unsigned count=0;
    for (unsigned id=1;id<=6;++id) {
        assert(!u6913_build_simple(&f, 1u<<(id-1), U6913_CMD_VERSION));
        for (unsigned seq=0;seq<256;++seq) {
            assert(!u6913_board_wrap(&f, seq%2, seq, &packet));
            unsigned previous=enqueues; output_len=0;
            assert(!command_process_frame(0,packet.data,packet.length));
            assert(enqueues==previous+1 && output_len==0);
            assert(queued.id==0 && queued.can_mode==CAN_MODE_FD && queued.frame_type==FRAME_TYPE_STD);
            assert(queued.dlc==64 && !memcmp(sent,f.data,64));
            ++count;
        }
    }
    for (unsigned n=12;n<=64;n+=4) {
        if (n!=12 && n!=32 && n!=48 && n!=64) continue;
        uint8_t payload[64]; for(unsigned i=0;i<n;++i)payload[i]=(uint8_t)(0xA0+i);
        CanMsg rx={.id=6,.frame_type=FRAME_TYPE_STD,.can_mode=CAN_MODE_FD,.dlc=n,.data=payload};
        assert(!can_frame_report(1,&rx));
        assert(!u6913_board_decode(output,output_len,&event));
        assert(event.kind==U6913_BOARD_CAN_RECEIVED && event.frame.can_id==6);
        assert(event.frame.len==n && !memcmp(event.frame.data,payload,n));
        ++count;
    }
    /* 真实固件返回的A0错误（重算CRC，使错误来自通道而非校验）。 */
    packet.data[7]=2;
    uint16_t c=crc16(packet.data,79); packet.data[79]=c>>8; packet.data[80]=c;
    command_process_frame(0,packet.data,packet.length);
    assert(!u6913_board_decode(output,output_len,&event));
    assert(event.kind==U6913_BOARD_REPLY && event.error_code==ERR_INVALID_PARAM && event.sequence==255);
    printf("PASS: %u request/push cases plus real firmware A0 error; no hardware accessed\n", count);
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--board-root", required=True, type=Path)
    args = parser.parse_args()
    board = args.board_root.resolve()
    sdk = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="u6913-board-") as temporary:
        work = Path(temporary)
        harness = work / "interop.c"
        harness.write_text(HARNESS)
        executable = work / "interop"
        sources = ["Core/Src/can_timing.c", "Core/Src/uart_stream_parser.c",
                   "Core/Src/command_process.c", "Core/Src/can_cmd_proc.c",
                   "Core/Src/base_cmd_proc.c", "Core/umbase/crc16.c"]
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
        for directory in (board / "tests/stubs", board / "tests", board / "Core/Inc",
                          board / "Core/umbase", sdk / "include"):
            command += ["-I", str(directory)]
        command += [str(harness)] + [str(board / source) for source in sources]
        command += [str(sdk / "controllers/uniubi6913.c"), str(sdk / "controllers/uniubi6913_board.c"),
                    "-o", str(executable)]
        subprocess.run(command, check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
