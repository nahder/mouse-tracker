from dlclivegui.camera import Camera, CameraError
import pyrealsense2 as rs
import numpy as np

class D435iCam(Camera):
    @staticmethod
    def arg_restrictions():
        return {'use_tk_display': [True, False],
                'resolution': [[1280, 720], [640, 480], [320, 240]]}

    def __init__(self, id="", 
                 resolution=[640, 480], 
                 exposure=0, 
                 crop=None, 
                 display_resize=1, 
                 use_tk_display=True):
        
        self.resolution = resolution
        self.exposure = exposure
        self.crop = crop
        self.display_resize = display_resize                           
        
        super().__init__(id, use_tk_display=use_tk_display, 
                         resolution=resolution, 
                         exposure=exposure, 
                         crop=crop,  
                         display_resize=display_resize)

    def set_capture_device(self):
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.color, self.resolution[0], self.resolution[1], rs.format.bgr8, 30)
        self.pipe.start(self.cfg)
        return True

    def get_image_on_time(self):
        frames = self.pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            raise CameraError("No frame available")
        color_image = np.asanyarray(color_frame.get_data())
        return color_image, frames.get_timestamp()
    
    def close_capture_device(self):
        self.pipe.stop()
    