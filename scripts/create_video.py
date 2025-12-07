import cv2
import os
import glob
import argparse
import sys

# --- 新增功能所需模块 ---
# 尝试导入ROS相关模块，如果失败则禁用ROS功能
try:
    import rosbag
    from cv_bridge import CvBridge, CvBridgeError

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    # 在主程序入口处打印警告，而不是在这里
    # print("警告：未找到ROS或cv_bridge。ROS bag提取功能将被禁用。")


def extract_images_from_bag(bag_path, output_root_dir, image_topic):
    """
    [新功能] 从ROS1 bag文件中提取指定topic的图像，并保存为PNG序列。

    参数:
    bag_path (str): .bag文件的路径。
    output_root_dir (str): 保存输出的根目录。图片将被保存在此目录下的一个新文件夹中。
    image_topic (str): 要提取的图像topic名称。

    返回:
    str: 保存了PNG序列的文件夹路径，如果失败则返回None。
    """
    if not ROS_AVAILABLE:
        print("错误：无法执行此功能，因为ROS环境不可用。")
        return None

    print("\n--- 开始从ROS Bag提取图像 ---")

    # 1. 检查bag文件是否存在
    if not os.path.exists(bag_path):
        print(f"错误：Bag文件不存在: {bag_path}")
        return None

    # 2. 创建输出目录
    bag_name = os.path.splitext(os.path.basename(bag_path))[0]
    # 清理topic名称，将'/'替换为'_'以创建有效的文件夹名 (移除开头的'/')
    safe_topic_name = image_topic.lstrip("/").replace("/", "_")
    image_output_dir = os.path.join(
        output_root_dir, f"{bag_name}_{safe_topic_name}_images"
    )
    os.makedirs(image_output_dir, exist_ok=True)
    print(f"图像将被保存到: {image_output_dir}")

    # 3. 初始化CvBridge并打开bag文件
    bridge = CvBridge()
    count = 0
    try:
        with rosbag.Bag(bag_path, "r") as bag:
            total_messages = bag.get_message_count(topic_filters=[image_topic])
            if total_messages == 0:
                print(f"错误：在bag文件中没有找到topic '{image_topic}' 的任何消息。")
                all_topics = bag.get_type_and_topic_info()[1].keys()
                if all_topics:
                    print("Bag中可用的Topics:")
                    for topic in sorted(all_topics):
                        print(f"- {topic}")
                return None

            print(
                f"在Topic '{image_topic}' 上找到 {total_messages} 条消息。开始提取..."
            )

            for topic, msg, t in bag.read_messages(topics=[image_topic]):
                try:
                    # 将ROS图像消息转换为OpenCV图像 (BGR格式)
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                except CvBridgeError as e:
                    print(f"CvBridge 错误: {e}")
                    continue

                # 使用消息的时间戳(纳秒)作为文件名，确保唯一性和正确排序
                timestamp_ns = msg.header.stamp.to_nsec()
                image_filename = os.path.join(image_output_dir, f"{timestamp_ns}.png")
                cv2.imwrite(image_filename, cv_image)
                count += 1

                if count % 100 == 0:
                    print(f"...已提取 {count} / {total_messages} 张图片")

    except rosbag.bag.ROSBagUnindexedException:
        print(
            f"错误：Bag文件未索引。请先在终端运行 'rosbag reindex {os.path.basename(bag_path)}'。"
        )
        return None
    except Exception as e:
        print(f"提取过程中发生未知错误: {e}")
        return None

    print(f"...已提取 {count} / {total_messages} 张图片")
    print("--- 图像提取完成 ---")
    return image_output_dir


def create_video_from_images(image_folder, output_dir, seq_name, fps=10):
    """
    (原始功能) 从指定的文件夹中读取按名称排序的图像，并合成为一个MP4视频。
    视频的左上角会标注当前帧对应的图片文件名，使用高对比度颜色确保清晰可见。

    参数:
    image_folder (str): 包含源图像文件的目录路径。
    output_dir (str): 用于保存输出视频的目录路径。
    seq_name (str): 数据集序列的名称，用于命名输出视频。
    fps (int): 输出视频的帧率。
    """
    print("\n--- 开始视频合成 ---")

    # 1. 检查图像文件夹是否存在
    if not os.path.isdir(image_folder):
        print(f"错误：图像文件夹不存在: {image_folder}")
        # sys.exit(1) # 改为return，使得主流程可以继续
        return

    # 构建输出视频的完整路径
    video_path = os.path.join(output_dir, f"{seq_name}.mp4")
    # 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)

    # 2. 查找所有.png, .jpg, .jpeg格式的图像文件
    print(f"正在文件夹 {image_folder} 中搜索 .png, .jpg, .jpeg 格式的图片...")
    image_files = []
    supported_formats = ("*.png", "*.jpg", "*.jpeg")
    for fmt in supported_formats:
        image_files.extend(glob.glob(os.path.join(image_folder, fmt)))

    if not image_files:
        print(f"警告：在 {image_folder} 中没有找到任何支持的图像文件。")
        return

    # 3. 按文件名进行升序排序
    image_files.sort()
    print(f"找到 {len(image_files)} 张图片进行处理。")

    # 4. 读取第一张图片以获取视频的尺寸
    try:
        first_frame = cv2.imread(image_files[0])
        if first_frame is None:
            raise IOError(f"无法读取第一张图片: {image_files[0]}")
        height, width, layers = first_frame.shape
        size = (width, height)
        print(f"视频尺寸将设置为: {width}x{height}")
    except Exception as e:
        print(f"错误：处理第一张图片时出错 - {e}")
        return

    # 5. 初始化VideoWriter
    try:
        video_writer = cv2.VideoWriter(
            video_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, size
        )
        if not video_writer.isOpened():
            raise IOError("无法打开 VideoWriter。请检查OpenCV和编解码器是否安装正确。")
    except Exception as e:
        print(f"错误：初始化VideoWriter失败 - {e}")
        return

    # 6. 循环读取图片并写入视频帧
    print("正在逐帧写入视频...")
    for i, filename in enumerate(image_files):
        img = cv2.imread(filename)
        if img is not None:
            # --- 文字标注设置 ---
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            text = os.path.basename(filename)
            position = (10, 30)  # 左上角位置

            # 颜色 (BGR格式)
            text_color_yellow = (0, 255, 255)  # 亮黄色
            outline_color_black = (0, 0, 0)  # 黑色

            # 厚度
            text_thickness = 2
            outline_thickness = 4  # 轮廓比文字稍粗

            # 先绘制黑色的轮廓（通过绘制稍粗的文字实现）
            cv2.putText(
                img,
                text,
                position,
                font,
                font_scale,
                outline_color_black,
                outline_thickness,
                cv2.LINE_AA,
            )

            # 再在上面绘制黄色的文字
            cv2.putText(
                img,
                text,
                position,
                font,
                font_scale,
                text_color_yellow,
                text_thickness,
                cv2.LINE_AA,
            )

            video_writer.write(img)

        # 打印进度
        if (i + 1) % 100 == 0:
            print(f"...已处理 {i + 1} / {len(image_files)} 帧")

    # 7. 释放资源
    video_writer.release()
    print(f"...已处理 {len(image_files)} / {len(image_files)} 帧")
    print("\n--- 视频合成成功！ ---")
    print(f"视频已保存至: {video_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="将图像序列或ROS bag中的图像合成为MP4视频。",
        formatter_class=argparse.RawTextHelpFormatter,  # 帮助信息格式更美观
    )
    script_dir = os.path.dirname(os.path.abspath(__file__))

    default_result_dir = os.path.join(script_dir, "../Log", "result")

    # --- 保留原有参数 ---
    parser.add_argument(
        "--result_dir",
        type=str,
        default=default_result_dir,
        help=f"用于保存视频和提取图像的目录。默认为: {default_result_dir}",
    )
    parser.add_argument(
        "--seq_name",
        type=str,
        default="output",
        help="数据集的序列名称，用于命名视频文件。",
    )
    parser.add_argument(
        "--fps", type=int, default=10, help="输出视频的帧率 (FPS)。默认为10。"
    )

    # --- 添加新功能的参数 ---
    if ROS_AVAILABLE:
        parser.add_argument(
            "--bag_path",
            type=str,
            default=None,
            help="[新功能] ROS1 .bag文件的路径。\n如果提供此参数，脚本将进入ROS提取模式。",
        )
        parser.add_argument(
            "--image_topic",
            type=str,
            default=None,
            help="[新功能] --bag_path所需，指定bag中的图像topic。",
        )
    else:
        print(
            "警告：未找到ROS或cv_bridge。ROS bag提取功能(--bag_path, --image_topic)已被禁用。"
        )
        print(
            "若要使用该功能，请确保您已正确安装ROS并配置了Python环境(例如 source /opt/ros/noetic/setup.bash)。"
        )

    args = parser.parse_args()

    # ===================================================================
    #                           执行逻辑
    # ===================================================================

    # 模式二: 从ROS Bag提取并创建视频 (如果提供了 --bag_path)
    if ROS_AVAILABLE and args.bag_path:
        if not args.image_topic:
            print("错误：使用 --bag_path 时，必须同时指定 --image_topic 参数。")
            sys.exit(1)

        # 1. 从bag提取图像。输出目录是result_dir，图片会保存在其中的一个新建子文件夹。
        extracted_image_folder = extract_images_from_bag(
            args.bag_path, args.result_dir, args.image_topic
        )

        # 2. 如果提取成功，则用提取出的图片合成视频
        if extracted_image_folder:
            create_video_from_images(
                image_folder=extracted_image_folder,
                output_dir=args.result_dir,
                seq_name=args.seq_name,
                fps=args.fps,
            )
        else:
            print("\n因图像提取失败，程序终止。")
            sys.exit(1)

    # 模式一: 从图像文件夹创建视频 (原始功能)
    else:
        # 如果用户试图使用ros功能但环境不可用，则提醒并退出
        if getattr(args, "bag_path", None):
            print("\n错误：您指定了 --bag_path，但ROS环境不可用。程序已终止。")
            sys.exit(1)

        print("--- 工作模式：从文件夹合成视频 ---")
        # 沿用原始脚本的逻辑来确定图片源文件夹
        is_default_path = os.path.abspath(args.result_dir) == os.path.abspath(
            default_result_dir
        )

        if is_default_path:
            # 如果使用的是默认路径，则在后面加上 'images' 子文件夹
            image_source_dir = os.path.join(args.result_dir, "images")
            print(f"使用默认图片路径: {image_source_dir}")
        else:
            # 如果用户指定了路径，则直接使用该路径作为图片源
            image_source_dir = args.result_dir
            print(f"使用用户指定图片路径: {image_source_dir}")

        # 视频的输出目录始终是 result_dir
        output_video_dir = args.result_dir

        # 调用原始函数
        create_video_from_images(
            image_source_dir, output_video_dir, args.seq_name, args.fps
        )


"""
python create_video.py \
    --bag_path /root/dataset/vio_inlab_0723.bag \
    --image_topic /camera/color/image_raw \
    --seq_name vio_video \
    --fps 30
"""
