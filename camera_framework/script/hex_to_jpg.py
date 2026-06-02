#!/usr/bin/env python3
"""
将包含十六进制数据的文本文件转换为JPG图片文件
支持解析多张JPG图片（通过检测 FFD8 起始标记和 FFD9 结束标记）
输入文件格式: 0xff,0xd8,0xff,0xe0, ...
"""

import re
import sys
import os


def find_jpg_images(byte_data):
    """
    在字节数据中查找所有JPG图片
    JPG图片以 FFD8 开始，以 FFD9 结束
    
    Args:
        byte_data: 字节数据
        
    Returns:
        list: 包含所有找到的JPG图片字节数据的列表
    """
    images = []
    start_marker = b'\xff\xd8'
    end_marker = b'\xff\xd9'
    
    pos = 0
    while pos < len(byte_data):
        # 查找JPG起始标记
        start_pos = byte_data.find(start_marker, pos)
        if start_pos == -1:
            break
        
        # 查找JPG结束标记
        end_pos = byte_data.find(end_marker, start_pos + 2)
        if end_pos == -1:
            # 没有找到结束标记，提取到末尾
            print(f"警告: 第 {len(images) + 1} 张图片没有找到结束标记 (FFD9)")
            images.append(byte_data[start_pos:])
            break
        
        # 提取完整的JPG数据（包含结束标记的两个字节）
        jpg_data = byte_data[start_pos:end_pos + 2]
        images.append(jpg_data)
        
        # 继续搜索下一张图片
        pos = end_pos + 2
    
    return images


def hex_to_jpg(input_file, output_file=None):
    """
    读取包含十六进制数据的文件并转换为JPG
    支持自动检测并分离多张JPG图片
    
    Args:
        input_file: 输入文件路径
        output_file: 输出JPG文件路径（可选，默认为输入文件名.jpg 或 输入文件名_1.jpg 等）
    """
    # 读取输入文件
    try:
        with open(input_file, 'r', encoding='utf-8') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"错误: 找不到文件 '{input_file}'")
        return False
    except Exception as e:
        print(f"错误: 读取文件时出错 - {e}")
        return False
    
    # 使用正则表达式提取所有十六进制数据
    # 匹配 0x 开头的十六进制数字
    hex_pattern = r'0x([0-9a-fA-F]{1,2})'
    hex_values = re.findall(hex_pattern, content)
    
    if not hex_values:
        print("错误: 文件中没有找到有效的十六进制数据")
        return False
    
    # 转换为字节数据
    try:
        byte_data = bytes([int(h, 16) for h in hex_values])
    except ValueError as e:
        print(f"错误: 十六进制数据转换失败 - {e}")
        return False
    
    print(f"读取到 {len(byte_data)} 字节数据")
    
    # 查找所有JPG图片
    images = find_jpg_images(byte_data)
    
    if not images:
        print("错误: 没有找到有效的JPG图片数据（缺少 FFD8 起始标记）")
        return False
    
    print(f"找到 {len(images)} 张JPG图片")
    
    # 生成基础文件名
    base_name = os.path.splitext(input_file)[0]
    if output_file:
        base_name = os.path.splitext(output_file)[0]
    
    # 写入JPG文件
    success_count = 0
    for i, img_data in enumerate(images):
        # 生成输出文件名
        if len(images) == 1:
            out_file = f"{base_name}.jpg"
        else:
            out_file = f"{base_name}_{i + 1}.jpg"
        
        try:
            with open(out_file, 'wb') as f:
                f.write(img_data)
            print(f"成功: 第 {i + 1} 张图片，{len(img_data)} 字节 -> '{out_file}'")
            success_count += 1
        except Exception as e:
            print(f"错误: 写入第 {i + 1} 张图片时出错 - {e}")
    
    print(f"\n完成: 成功保存 {success_count}/{len(images)} 张图片")
    return success_count > 0


def main():
    if len(sys.argv) < 2:
        print("用法: python hex_to_jpg.py <输入文件> [输出文件前缀]")
        print("示例: python hex_to_jpg.py input.txt output")
        print("      python hex_to_jpg.py input.txt  (自动生成 input.jpg 或 input_1.jpg, input_2.jpg ...)")
        print("\n说明: 支持自动检测并分离多张JPG图片")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    success = hex_to_jpg(input_file, output_file)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
