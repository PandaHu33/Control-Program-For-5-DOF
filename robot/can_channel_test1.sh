#!/bin/bash
INTERFACE="can0"          # 你的CAN接口
BITRATE="1000000"          # 波特率，按实际修改
TARGET_IDS=(1 2 3 4 5)    # 要测试的终端ID
TIMEOUT_MS=100            # 等待回复的超时时间(毫秒)
ROUNDS=100                # 每个终端测试轮次

# 配置CAN接口
sudo ip link set $INTERFACE down
sudo ip link set $INTERFACE type can bitrate $BITRATE
sudo ip link set $INTERFACE up

# 统计数组
declare -A SENT RECEIVED LOST

for id in ${TARGET_IDS[@]}; do
    SENT[$id]=0
    RECEIVED[$id]=0
done

echo "开始测试, 每终端 $ROUNDS 轮, 超时 ${TIMEOUT_MS}ms"
echo "终端ID | 发送 | 收到 | 丢包率 | 平均响应(ms)"
echo "-------|------|------|--------|-------------"

for id in ${TARGET_IDS[@]}; do
    LOSS=0
    TOTAL_RT=0
    SUCCESS=0

    for ((round=1; round<=$ROUNDS; round++)); do
        # 发送查询帧 (ID=主控自定如0x600, 数据=请求终端ID)
        cansend $INTERFACE 600#$(printf "%02x" $id)
        ((SENT[$id]++))

        # 接收回复，只关心来自当前测试终端ID的帧
        START=$(date +%s%N)
        REPLY=$(candump $INTERFACE -L -n 1 -t a 2>/dev/null | grep "  $id#" | head -1)
        END=$(date +%s%N)

        if [ -n "$REPLY" ]; then
            # 计算响应时间(毫秒)
            RT_MS=$(( ($END - $START) / 1000000 ))
            TOTAL_RT=$((TOTAL_RT + RT_MS))
            ((RECEIVED[$id]++))
            ((SUCCESS++))
        fi
    done

    LOST=$((SENT[$id] - RECEIVED[$id]))
    LOSS_RATE=$(echo "scale=1; $LOST * 100 / $SENT[$id]" | bc)
    AVG_RT=$(echo "scale=1; $TOTAL_RT / $SUCCESS" | bc)

    printf "  %d    |  %3d  |  %3d  |  %5.1f%%  |     %5.1f\n" \
           $id ${SENT[$id]} ${RECEIVED[$id]} $LOSS_RATE $AVG_RT
done

echo "测试完成"
