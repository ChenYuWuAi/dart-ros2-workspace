#!/bin/bash

# 本脚本用于从给定的BV号从B站下载音乐文件，同时根据命令行参数决定是否播放
# 使用示例：./fetch_music.sh BV1xxxxxx --play(可选) --volume 50(可选)

# 决定media-get工具的路径
# Architecture检查
if [ "$(uname -m)" == "x86_64" ]; then
    MEDIA_GET_PATH="./bin/media-get-x86_64"
elif [ "$(uname -m)" == "aarch64" ]; then
    MEDIA_GET_PATH="./bin/media-get-aarch64"
else
    echo "不支持的架构：$(uname -m)。请使用x86_64或aarch64架构。"
    exit 1
fi

# 检查是否安装了必要的工具
if ! command -v $MEDIA_GET_PATH &>/dev/null; then
    echo "脚本调用目录错误，未找到 media-get 命令。请确保已安装 media-get 工具。"
    exit 1
fi

# 检查是否提供了BV号
if [ -z "$1" ]; then
    echo "请提供B站视频的BV号。"
    exit 1
fi

BV_ID="$1"
PLAY_FLAG=false
# 默认音量为50%
VOLUME=50

# 解析参数，支持 --play/-p 和 --volume/-v
i=2
while [ $i -le $# ]; do
    arg="${!i}"
    case "$arg" in
        --play|-p)
            PLAY_FLAG=true
            ;;
        --volume|-v)
            next_idx=$((i+1))
            next_arg="${!next_idx}"
            if [[ -n "$next_arg" && "$next_arg" =~ ^[0-9]+$ && "$next_arg" -ge 0 && "$next_arg" -le 100 ]]; then
                VOLUME="$next_arg"
                i=$((i+1)) # 跳过下一个参数
            else
                echo "音量参数无效，请提供0到100之间的整数。"
                exit 1
            fi
            ;;
    esac
    i=$((i+1))
done

# 检查是否存在音乐目录，如果不存在则创建
if [ ! -d "./music" ]; then
    mkdir -p "./music"
fi
# 检查BV号格式
if [[ ! "$BV_ID" =~ ^BV[a-zA-Z0-9]{10}$ ]]; then
    echo "无效的BV号格式，请提供正确的BV号。"
    exit 1
fi
# 检查是否已经下载过该BV号的音乐文件
if [ -f "./music/$BV_ID.mp3" ]; then
    echo "该BV号的音乐文件已经下载过，跳过下载。"
else
    echo "开始下载BV号为 $BV_ID 的音乐文件..."

    # 下载音乐文件
    echo "正在下载BV号为 $BV_ID 的音乐文件..."
    HTTPS_PATH="https://www.bilibili.com/video/$BV_ID"
    $MEDIA_GET_PATH -u "$HTTPS_PATH" -o "./music/$BV_ID.mp3" -t audio

    # 检查下载是否成功
    if [ $? -ne 0 ]; then
        echo "下载失败，请检查BV号或网络连接。"
        exit 1
    fi
fi

# 如果有 --play 参数，播放音乐文件
if $PLAY_FLAG; then
    echo "正在播放音乐文件..."

    # 设置音量
    if command -v amixer &>/dev/null; then
        amixer set Master "$VOLUME"% >/dev/null
    else
        echo "未找到amixer命令，无法设置音量。"
    fi
    # 播放音乐文件
    if command -v mpv &>/dev/null; then
        mpv --no-video "./music/$BV_ID.mp3"
    elif command -v vlc &>/dev/null; then
        cvlc "./music/$BV_ID.mp3"
    else
        echo "未找到可用的音频播放器，请安装mpv或vlc。"
        exit 1
    fi
else
    echo "下载完成，音乐文件已保存到 ./music/$BV_ID.mp3"
fi
# 结束脚本
exit 0
