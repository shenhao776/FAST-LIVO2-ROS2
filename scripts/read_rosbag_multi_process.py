#!/usr/bin/python3

import cv2
import os
import glob
import argparse
import sys
import multiprocessing
import numpy as np

# --- 模块导入与ROS环境检查 ---
try:
    import rosbag
    from cv_bridge import CvBridge, CvBridgeError
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

def worker_extract_chunk(process_id, bag_path, image_output_dir, image_topic, timestamp_chunk_set):
    """
    [工作进程函数]
    每个进程执行此函数，负责提取其被分配到的一部分图像。

    参数:
    process_id (int): 进程编号，用于日志区分。
    bag_path (str): .bag文件的路径。
    image_output_dir (str): 保存提取图像的目标文件夹。
    image_topic (str): 要提取的图像topic名称。
    timestamp_chunk_set (set): 一个包含此进程需要提取的所有图像时间戳的集合。
    """
    print(f"[进程 {process_id}] 已启动。负责 {len(timestamp_chunk_set)} 张图片。")
    
    bridge = CvBridge()
    count = 0
    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            # 每个进程都完整迭代一遍bag文件
            for topic, msg, t in bag.read_messages(topics=[image_topic]):
                timestamp_ns = msg.header.stamp.to_nsec()
                # 检查当前消息的时间戳是否在自己的任务列表里
                if timestamp_ns in timestamp_chunk_set:
                    try:
                        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                        image_filename = os.path.join(image_output_dir, f"{timestamp_ns}.png")
                        cv2.imwrite(image_filename, cv_image)
                        count += 1
                    except CvBridgeError as e:
                        print(f"[进程 {process_id}] CvBridge 错误: {e}")
                        continue
    except Exception as e:
        print(f"[进程 {process_id}] 发生未知错误: {e}")

    print(f"[进程 {process_id}] 完成。成功提取 {count} 张图片。")


def create_video_from_images(image_folder, video_path, fps=10):
    """
    从指定的文件夹中读取图像，并合成为一个MP4视频。
    """
    print(f"\n[主进程] 开始从 {image_folder} 合成视频...")

    if not os.path.isdir(image_folder):
        print(f"[主进程] 错误 - 图像文件夹不存在: {image_folder}")
        return

    image_files = []
    supported_formats = ('*.png', '*.jpg', '*.jpeg', '*.bmp', '*.tiff')
    for fmt in supported_formats:
        image_files.extend(glob.glob(os.path.join(image_folder, fmt)))

    if not image_files:
        print(f"[主进程] 警告 - 在 {image_folder} 中没有找到任何支持的图像文件。")
        return

    image_files.sort()
    print(f"[主进程] 找到 {len(image_files)} 张图片进行处理。")

    try:
        first_frame = cv2.imread(image_files[0])
        if first_frame is None: raise IOError(f"无法读取第一张图片: {image_files[0]}")
        height, width, _ = first_frame.shape
        size = (width, height)
    except Exception as e:
        print(f"[主进程] 错误 - 处理第一张图片时出错: {e}")
        return

    try:
        os.makedirs(os.path.dirname(video_path), exist_ok=True)
        video_writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
        if not video_writer.isOpened(): raise IOError("无法打开 VideoWriter。")
    except Exception as e:
        print(f"[主进程] 错误 - 初始化VideoWriter失败: {e}")
        return

    for i, filename in enumerate(image_files):
        img = cv2.imread(filename)
        if img is not None:
            font = cv2.FONT_HERSHEY_SIMPLEX
            text = os.path.basename(filename)
            position = (10, 30)
            text_color_yellow = (0, 255, 255)
            outline_color_black = (0, 0, 0)
            cv2.putText(img, text, position, font, 0.7, outline_color_black, 4, cv2.LINE_AA)
            cv2.putText(img, text, position, font, 0.7, text_color_yellow, 2, cv2.LINE_AA)
            video_writer.write(img)
        
        if (i + 1) % 500 == 0:
            print(f"[主进程] ...已合成 {i + 1} / {len(image_files)} 帧")

    video_writer.release()
    print(f"[主进程] --- 视频合成成功！视频已保存至: {video_path} ---")


if __name__ == '__main__':
    # 确保在Windows或macOS上使用 'spawn' 或 'forkserver' 启动方式
    if sys.platform.startswith('win') or sys.platform.startswith('darwin'):
        multiprocessing.set_start_method('spawn', force=True)

    parser = argparse.ArgumentParser(
        description='使用多进程并行处理一个大的ROS bag，提取图像并合成为MP4视频。',
        formatter_class=argparse.RawTextHelpFormatter
    )
    
    parser.add_argument('--bag_path', type=str, required=True,
                        help="[必需] 单个大的ROS1 .bag文件的路径。")
    parser.add_argument('--output_dir', type=str, default="results",
                        help="用于保存所有输出的总目录。默认为 'results'。")
    parser.add_argument('--image_topic', type=str, required=True,
                        help="[必需] bag中要提取的图像topic。")
    parser.add_argument('--fps', type=int, default=30,
                        help="输出视频的帧率 (FPS)。默认为30。")
    parser.add_argument('--workers', type=int, default=None,
                        help="用于处理的进程数。默认为系统CPU核心数。")

    if not ROS_AVAILABLE:
        print("="*60 + "\n警告：未找到ROS或cv_bridge。脚本无法运行。\n" + "="*60)
        sys.exit(1)

    args = parser.parse_args()

    # ===================================================================
    #                           多进程执行逻辑
    # ===================================================================
    
    # 1. 定义输出路径
    bag_name = os.path.splitext(os.path.basename(args.bag_path))[0]
    bag_output_dir = os.path.join(args.output_dir, bag_name)
    image_folder = os.path.join(bag_output_dir, "images")
    video_path = os.path.join(bag_output_dir, f"{bag_name}.mp4")
    os.makedirs(image_folder, exist_ok=True)

    # 2. 预扫描，获取所有时间戳
    print(f"[主进程] 步骤 1/4: 正在预扫描 {bag_name}.bag 以索引所有图像时间戳...")
    try:
        with rosbag.Bag(args.bag_path, 'r') as bag:
            total_messages = bag.get_message_count(topic_filters=[args.image_topic])
            if total_messages == 0:
                print(f"错误：在bag文件中没有找到topic '{args.image_topic}' 的任何消息。")
                sys.exit(1)
            
            all_timestamps = [msg.header.stamp.to_nsec() for _, msg, _ in bag.read_messages(topics=[args.image_topic])]
    except Exception as e:
        print(f"预扫描bag文件时出错: {e}")
        sys.exit(1)
        
    print(f"[主进程] 预扫描完成。共找到 {len(all_timestamps)} 张图片。")

    # 3. 分块并启动多进程
    num_workers = args.workers if args.workers else os.cpu_count()
    # 使用numpy.array_split来尽可能均匀地分割列表
    timestamp_chunks = np.array_split(all_timestamps, num_workers)
    
    # 将任务参数打包，注意要将每个块转为set以提高查找效率
    tasks = [
        (i+1, args.bag_path, image_folder, args.image_topic, set(chunk))
        for i, chunk in enumerate(timestamp_chunks) if len(chunk) > 0
    ]
    
    print(f"\n[主进程] 步骤 2/4: 将任务分割成 {len(tasks)} 块，启动 {len(tasks)} 个工作进程...")
    
    with multiprocessing.Pool(processes=len(tasks)) as pool:
        pool.starmap(worker_extract_chunk, tasks)

    print("\n[主进程] 步骤 3/4: 所有工作进程已完成图像提取。")

    # 4. 合成视频
    create_video_from_images(image_folder, video_path, args.fps)
    
    print("\n[主进程] 步骤 4/4: 所有任务已完成。")

"""
=========================
        示例用法
=========================

# 使用所有可用的CPU核心处理一个大的bag文件
python3 create_video.py \
    --bag_path /root/dataset/vio_daiwa_f2_0724.bag \
    --image_topic /camera/color/image_raw \
    --output_dir /root \
    --fps 30

# 指定使用4个CPU核心
python3 create_video.py \
    --bag_path /path/to/your/large_file.bag \
    --image_topic /camera/color/image_raw \
    --output_dir /path/to/my_results \
    --fps 30 \
    --workers 4
"""
