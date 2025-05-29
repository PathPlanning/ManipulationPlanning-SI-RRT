r"""
Video Comparison Tool for Multiple Planning Algorithms

This script creates comparison videos showing the results of three different
motion planning algorithms (ST-RRT, SI-RRT and DRGBT) side-by-side. 
It combines videos from each algorithm into a single composite video,
adding status labels that indicate success/failure and path cost.

The script takes video files as input, along with metadata to display
the success or failure of each algorithm's planning attempt.

Example of use:

python3 ./Blender/scripts/build_video.py --file_strrt visualisation_of_STRRT*_planner_logs_1746262528_-1830845017.mp4 \
    --file_sirrt visualisation_of_MSIRRT_planner_logs_1746262528_-1830845017.mp4 \
    --file_drgbt visualisation_of_DRGBT_planner_logs_1746262528_-1830845017.mp4 \
    --dir_video ./mass_test/tests/1_spheres/test_number_0 \
    --output_file output.mp4


python3 ./Blender/scripts/build_video.py --file_strrt visualisation_of_STRRT*_planner_logs_1746260598_1972524557.mp4 \
    --file_sirrt visualisation_of_MSIRRT_planner_logs_1746260597_935763300.mp4 \
    --file_drgbt visualisation_of_DRGBT_planner_logs_1746262669_-673011725.mp4 \
    --dir_video tests/tests/40_spheres/test_number_1 \
    --output_file output.mp4

python3 ./Blender/scripts/build_video.py --dir_video ./ --output_file output.mp4


"""

import subprocess
import ffmpeg
import os
import argparse
from pathlib import Path
import json
import platform
from typing import Tuple, List, Union, Dict, Any, Optional

def get_video_frame_count(path: str) -> int:
    """
    Count the number of frames in a video file.
    
    Args:
        path: Path to the video file.
        
    Returns:
        Number of frames in the video.
    """
    cmd: List[str] = [
        'ffprobe', '-v', 'error',
        '-select_streams', 'v:0',
        '-count_frames',
        '-show_entries', 'stream=nb_read_frames',
        '-print_format', 'default=nokey=1:noprint_wrappers=1',
        path
    ]
    output: str = subprocess.check_output(cmd).decode().strip()
    return int(output)

def get_fps(path: str) -> float:
    """
    Get the frames per second (fps) of a video file.
    
    Args:
        path: Path to the video file.
        
    Returns:
        The frames per second as a float.
    """
    cmd: List[str] = [
        'ffprobe', '-v', 'error',
        '-select_streams', 'v:0',
        '-show_entries', 'stream=r_frame_rate',
        '-of', 'default=noprint_wrappers=1:nokey=1',
        path
    ]
    fps_str: str = subprocess.check_output(cmd).decode().strip()
    num, den = fps_str.split('/')
    return float(num) / float(den)

def add_status_text(input_video: Any, duration: float, success: bool, cost: Optional[float], fontfile: str, fontsize: int = 100, x: int = 10, y: int = 10) -> Any:
    """
    Add status text overlay to a video based on the success status.
    
    Args:
        input_video: ffmpeg video stream to add text to.
        duration: Length of the video in seconds.
        success: Whether the planner reached its goal successfully.
        cost: Path cost (time in seconds) if successful.
        fontfile: Path to the font file to use for text.
        fontsize: Size of the font in pixels.
        x: X-coordinate for text position.
        y: Y-coordinate for text position.
        
    Returns:
        Modified ffmpeg video stream with text overlay.
    """
    if success:
        text: str = f"Reached goal in {cost:.2f} s"
        fontcolor: str = "Green"
        borderw: int = 3
    else:
        text: str = "Collided with an obstacle"
        fontcolor: str = "Red"
        borderw: int = 5

    enable_expr: str = f"between(t,{duration - 0.05},{duration})"

    return input_video.drawtext(
        text=text,
        fontfile=fontfile,
        fontsize=fontsize,
        fontcolor=fontcolor,
        x=x,
        y=y,
        borderw=borderw,
        bordercolor='black',
        enable=enable_expr,
        shadowcolor='black',
        shadowx=2,
        shadowy=2
    )
    
def main(videodrgbt_path: Optional[str], videosirrt_path: Optional[str], videostrrt_path: Optional[str], 
         success_drgbt: bool, cost_drgbt: Optional[float], success_sirrt: bool, cost_si_rrt: Optional[float], 
         success_strrt: bool, cost_strrt: Optional[float], output_path: str = 'output.mp4') -> None:
    """
    Create a comparison video showing results from three different planners.
    
    This function takes three video files, each representing the output of a different
    planner (ST-RRT, SI-RRT and DRGBT), and combines them into a single video for comparison.
    The output is a video with DRGBT at the top and SI-RRT and ST-RRT* at the bottom.
    Status text is overlaid on each video showing whether the planner was successful
    and how long it took.
    
    Args:
        videodrgbt_path: Path to the DRGBT video file. If None, a black background will be used.
        videosirrt_path: Path to the SI-RRT video file. If None, a black background will be used.
        videostrrt_path: Path to the ST-RRT* video file. If None, a black background will be used.
        success_drgbt: Whether the DRGBT planner reached its goal successfully.
        cost_drgbt: Path cost for the DRGBT planner (time in seconds).
        success_sirrt: Whether the SI-RRT planner reached its goal successfully.
        cost_si_rrt: Path cost for the SI-RRT planner (time in seconds).
        success_strrt: Whether the ST-RRT* planner reached its goal successfully.
        cost_strrt: Path cost for the ST-RRT* planner (time in seconds).
        output_path: Path for the output comparison video file.
    """
    # Default dimensions and fps
    default_width: int = 960
    default_height: int = 540
    default_fps: float = 30.0
    default_frames: int = 30  # 1 second at 30 fps
    default_duration: float = default_frames / default_fps

    # Get frame counts and fps from available videos
    frames1: int = get_video_frame_count(videodrgbt_path) if videodrgbt_path else default_frames
    frames2: int = get_video_frame_count(videosirrt_path) if videosirrt_path else default_frames
    frames3: int = get_video_frame_count(videostrrt_path) if videostrrt_path else default_frames
    max_frames: int = max(frames1, frames2, frames3)
    
    # Determine fps from available videos, or use default
    fps1: float = 0.0
    if videodrgbt_path:
        fps1 = get_fps(videodrgbt_path)
    elif videosirrt_path:
        fps1 = get_fps(videosirrt_path)
    elif videostrrt_path:
        fps1 = get_fps(videostrrt_path)
    else:
        fps1 = default_fps
    

    def get_pad_duration(frames: int, max_frames: int, fps: float) -> float:
        """
        Calculate padding duration needed to extend a video to match the maximum frame count.
        
        Args:
            frames: Current frame count of the video.
            max_frames: Target frame count (maximum of all videos).
            fps: Frames per second of the video.
            
        Returns:
            Duration in seconds to pad the video.
        """
        return max(1, (max_frames - frames) / fps)

    # Calculate padding durations for each video
    pad1: float = get_pad_duration(frames1, max_frames, fps1) if videodrgbt_path else 0
    pad2: float = get_pad_duration(frames2, max_frames, fps1) if videosirrt_path else 0
    pad3: float = get_pad_duration(frames3, max_frames, fps1) if videostrrt_path else 0

    # Set up the font based on platform
    if platform.system() == "Windows":
        fontfile: str = "C:/Windows/Fonts/arial.ttf"
    elif platform.system() == "Linux":
        fontfile: str = "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"
    elif platform.system() == "Darwin":  # macOS
        fontfile: str = "/System/Library/Fonts/Arial.ttf"
    else:
        raise OSError("Didn't recognise OS, can't find font file")

    # Create input streams for available videos or black backgrounds
    duration_str: str = str(max_frames / fps1)
    
    # Create input videos or black backgrounds
    if videodrgbt_path:
        input1 = ffmpeg.input(videodrgbt_path)
        input1 = add_status_text(input1, frames1/fps1, success_drgbt, cost_drgbt, fontfile, fontsize=72, x=10, y=10)
        v1 = input1.filter('tpad', stop_mode='clone', stop_duration=pad1)
        probe1: Dict[str, Any] = ffmpeg.probe(videodrgbt_path)
        w1: int = int(probe1['streams'][0]['width'])
        h1: int = int(probe1['streams'][0]['height'])
    else:
        # Create black background if no video
        w1, h1 = default_width, default_height
        v1 = ffmpeg.input(f'color=black:s={w1}x{h1}:d={duration_str}', f='lavfi')
        
    if videosirrt_path:
        input2 = ffmpeg.input(videosirrt_path)
        input2 = add_status_text(input2, frames2/fps1, success_sirrt, cost_si_rrt, fontfile, fontsize=72, x=10, y=10)
        v2 = input2.filter('tpad', stop_mode='clone', stop_duration=pad2)
        probe2: Dict[str, Any] = ffmpeg.probe(videosirrt_path)
        w2_orig: int = int(probe2['streams'][0]['width'])
        h2_orig: int = int(probe2['streams'][0]['height'])
    else:
        # Create black background if no video
        w2_orig, h2_orig = default_width, default_height
        v2 = ffmpeg.input(f'color=black:s={w2_orig}x{h2_orig}:d={duration_str}', f='lavfi')

    if videostrrt_path:
        input3 = ffmpeg.input(videostrrt_path)
        input3 = add_status_text(input3, frames3/fps1, success_strrt, cost_strrt, fontfile, fontsize=72, x=10, y=10)
        v3 = input3.filter('tpad', stop_mode='clone', stop_duration=pad3)
        probe3: Dict[str, Any] = ffmpeg.probe(videostrrt_path)
        w3_orig: int = int(probe3['streams'][0]['width'])
        h3_orig: int = int(probe3['streams'][0]['height'])
    else:
        # Create black background if no video
        w3_orig, h3_orig = default_width, default_height
        v3 = ffmpeg.input(f'color=black:s={w3_orig}x{h3_orig}:d={duration_str}', f='lavfi')

    # # Scale lower videos to h1 height, preserving aspect ratio
    # v2 = v2.filter('scale', -1, h1)
    # v3 = v3.filter('scale', -1, h1)

    # Calculate width of scaled lower videos
    w2: int = int(w2_orig * h1 / h2_orig)
    w3: int = int(w3_orig * h1 / h3_orig)

    # Final dimensions of the output video
    final_w: int = max(w1, w2 + w3)
    final_h: int = h1 * 2

    # Create black background for composite video
    background = ffmpeg.input(f'color=black:s={final_w}x{final_h}:d={duration_str}', f='lavfi')

    # Place DRGBT at the top center
    x1: int = (final_w - w1) // 2
    overlay1 = ffmpeg.overlay(background, v1, x=x1, y=0)

    # Place SI-RRT at the bottom left
    overlay2 = ffmpeg.overlay(overlay1, v2, x=0, y=h1)

    # Place ST-RRT at the bottom right
    overlay3 = ffmpeg.overlay(overlay2, v3, x=final_w - w3, y=h1)
    
    # After creating overlay3 (final video with three videos on black background), add drawtext for each label
    fontsize: int = 100

    # Add "DRGBT:" label at the top left
    x1 = (final_w - w1) // 2 
    drgbt_text_x: int = max(x1 - fontsize*3-80, 10)
    drgbt_text_y: int = 10

    result = overlay3.drawtext(
        text='DRGBT:',
        fontfile=fontfile,
        fontsize=fontsize,
        fontcolor='white',
        x=drgbt_text_x,
        y=drgbt_text_y,
        shadowcolor='black',
        shadowx=2,
        shadowy=2
    )

    # Add "DRGBT: did not solve" if no video
    if not videodrgbt_path:
        drgbt_status_x: int = (final_w) // 2 - 200
        drgbt_status_y: int = h1 // 2
        result = result.drawtext(
            text='DRGBT: did not solve',
            fontfile=fontfile,
            fontsize=fontsize//2,
            fontcolor='white',
            x=drgbt_status_x,
            y=drgbt_status_y,
            shadowcolor='black',
            shadowx=2,
            shadowy=2
        )

    # Add "SI-RRT" label at the bottom left
    si_rrt_text_x: int = 10
    si_rrt_text_y: int = h1 - 80

    result = result.drawtext(
        text='SI-RRT:',
        fontfile=fontfile,
        fontsize=fontsize,
        fontcolor='white',
        x=si_rrt_text_x,
        y=si_rrt_text_y,
        shadowcolor='black',
        shadowx=2,
        shadowy=2
    )

    # Add "SI-RRT: did not solve" if no video
    if not videosirrt_path:
        si_rrt_status_x: int = w2 // 2 - 200
        si_rrt_status_y: int = h1 + h1 // 2
        result = result.drawtext(
            text='SI-RRT: did not solve',
            fontfile=fontfile,
            fontsize=fontsize//2,
            fontcolor='white',
            x=si_rrt_status_x,
            y=si_rrt_status_y,
            shadowcolor='black',
            shadowx=2,
            shadowy=2
        )

    # Add "ST-RRT*" label at the top right
    st_rrt_text_x: int = final_w*3/4 + 50
    st_rrt_text_y: int = h1 - 80

    result = result.drawtext(
        text='ST-RRT*:',
        fontfile=fontfile,
        fontsize=fontsize,
        fontcolor='white',
        x=st_rrt_text_x,
        y=st_rrt_text_y,
        shadowcolor='black',
        shadowx=2,
        shadowy=2
    )

    # Add "ST-RRT*: did not solve" if no video
    if not videostrrt_path:
        st_rrt_status_x: int = final_w - w3 // 2 - 200
        st_rrt_status_y: int = h1 + h1 // 2
        result = result.drawtext(
            text='ST-RRT*: did not solve',
            fontfile=fontfile,
            fontsize=fontsize//2,
            fontcolor='white',
            x=st_rrt_status_x,
            y=st_rrt_status_y,
            shadowcolor='black',
            shadowx=2,
            shadowy=2
        )

    
    
    
    out = ffmpeg.output(
            result, output_path,
            vcodec='libx264',
            pix_fmt='yuv420p',
            r=fps1
        )

    out = out.global_args('-loglevel', 'info')
    ffmpeg.run(out)


if __name__ == '__main__':
    """
    Main script entry point.
    
    Parses command line arguments, reads planner result metadata from JSON files,
    and creates a comparison video.
    """
    parser = argparse.ArgumentParser(description='Build a comparison video from planner results')
    parser.add_argument('--file_strrt', type=str, default='visualisation_of_STRRT__planner_logs_1746260597_-1673350316.mp4',
                        help='Path to ST-RRT* video file')
    parser.add_argument('--file_sirrt', type=str, default='visualisation_of_MSIRRT_planner_logs_1746260596_-292961663.mp4',
                        help='Path to SI-RRT video file')
    parser.add_argument('--file_drgbt', type=str, default='visualisation_of_DRGBT_planner_logs_1746262891_1924790686.mp4',
                        help='Path to DRGBT video file')
    parser.add_argument('--dir_video', type=str, default='./s2',
                        help='Directory containing video files and JSON metadata')
    parser.add_argument('--output_file', type=str, default=None,
                        help='Output file path for the comparison video (default: dir_video+".mp4")')
    args = parser.parse_args()
    
    file_strrt: str = args.file_strrt
    file_sirrt: str = args.file_sirrt
    file_drgbt: str = args.file_drgbt
    dir_video: str = args.dir_video
    
    # Initialize default values for success and path_cost
    success_strrt: bool = False
    path_cost_strrt: Optional[float] = None
    success_sirrt: bool = False
    path_cost_sirrt: Optional[float] = None
    success_drgbt: bool = False
    path_cost_drgbt: Optional[float] = None
    
    # Compute file paths, setting to None if file doesn't exist
    video_strrt: Optional[str] = os.path.join(dir_video, file_strrt)
    if not os.path.exists(video_strrt):
        video_strrt = None
        
    video_sirrt: Optional[str] = os.path.join(dir_video, file_sirrt)
    if not os.path.exists(video_sirrt):
        video_sirrt = None
        
    video_drgbt: Optional[str] = os.path.join(dir_video, file_drgbt)
    if not os.path.exists(video_drgbt):
        video_drgbt = None
    
    # Load metadata for available videos
    if video_strrt:
        try:
            with open(os.path.join(dir_video, Path(file_strrt).stem[17:]+'.json'), 'r', encoding='utf-8') as f:
                data: Dict[str, Any] = json.load(f)
                success_strrt = data.get('final_planner_data', {}).get('has_result', False)
                path_cost_strrt = data.get('final_planner_data', {})['final_path'][-1]['time']
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Warning: Couldn't load ST-RRT metadata: {e}")
    
    if video_sirrt:
        try:
            with open(os.path.join(dir_video, Path(file_sirrt).stem[17:]+'.json'), 'r', encoding='utf-8') as f:
                data: Dict[str, Any] = json.load(f)
                success_sirrt = data.get('final_planner_data', {}).get('has_result', False)
                path_cost_sirrt = data.get('final_planner_data', {})['final_path'][-1]['time']
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Warning: Couldn't load SI-RRT metadata: {e}")
    
    if video_drgbt:
        try:
            with open(os.path.join(dir_video, Path(file_drgbt).stem[17:]+'.json'), 'r', encoding='utf-8') as f:
                data: Dict[str, Any] = json.load(f)
                success_drgbt = data.get('final_planner_data', {}).get('has_result', False)
                path_cost_drgbt = data.get('final_planner_data', {})['final_path'][-1]['time']
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Warning: Couldn't load DRGBT metadata: {e}")
    
    # Determine output file path
    output_path: str = args.output_file if args.output_file else dir_video+'.mp4'
    print(success_strrt, path_cost_strrt, 
         success_sirrt, path_cost_sirrt, 
         success_drgbt, path_cost_drgbt)
    # Call main function to create the video
    main(video_drgbt, video_sirrt, video_strrt, 
         success_drgbt, path_cost_drgbt, 
         success_sirrt, path_cost_sirrt, 
         success_strrt, path_cost_strrt,
         output_path=output_path)
