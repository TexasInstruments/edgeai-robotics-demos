#  Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""EdgeAI post-processing

This script provides the post-processing logic for the EdgeAI based object detection
chain.

"""

from classnames import *
import cv2
import numpy as np
import copy
import debug

np.set_printoptions(threshold=np.inf, linewidth=np.inf)

def create_title_frame(title, width, height):
    frame = np.zeros((height, width, 3), np.uint8)
    frame = cv2.putText(frame, "Texas Instruments - Edge Analytics", \
                    (40, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 2)
    frame = cv2.putText(frame, title, (40, 70), \
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    return frame

def overlay_model_name(frame, model_name, start_x, start_y, width, height):
    row_size = 40 * width//1280
    font_size = width/1280
    cv2.putText(frame, "Model : " + model_name, \
                (start_x + 5, start_y - row_size//4), cv2.FONT_HERSHEY_SIMPLEX, \
                font_size, (255, 255, 255), 2)
    return frame

class PostProcess:
    """
    Class to create a post process context
    """
    def __init__(self, flow):
        self.flow = flow
        self.model = flow.model
        self.debug = None
        self.debug_str = ""
        if flow.debug_config and flow.debug_config.post_proc:
            self.debug = debug.Debug(flow.debug_config, "post")

    def get(flow, cbObj):
        """
        Create a object of a subclass based on the task type
        """
        if (flow.model.task_type == "detection"):
            return PostProcessTracking(flow, cbObj)
        else:
            return None

class PostProcessTracking(PostProcess):
    def __init__(self, flow, cbObj):
        super().__init__(flow)
        self.classnames = eval(flow.model.dataset)

        self.cbObj = cbObj

    def __call__(self, img, results):
        """
        Post process function for detection
        Args:
            img: Input frame
            results: output of inference
        """
        for i,r in enumerate(results):
            r = np.squeeze(r)
            if r.ndim == 1:
                r = np.expand_dims(r, 1)
            results[i] = r

        if self.model.shuffle_indices:
            results_reordered = []
            for i in self.model.shuffle_indices:
                results_reordered.append(results[i])
            results = results_reordered

        if results[-1].ndim < 2:
            results = results[:-1]

        bbox = np.concatenate(results, axis = -1)

        if self.model.formatter:
            bbox_copy = copy.deepcopy(bbox)
            bbox[..., self.model.formatter["dst_indices"]] = \
                             bbox_copy[..., self.model.formatter["src_indices"]]

        if not self.model.normalized_detections:
            bbox[..., (0, 2)] /= self.model.resize[0]
            bbox[..., (1, 3)] /= self.model.resize[1]

        box = None
        reset = True
        class_name = None
        for b in bbox:
            if self.cbObj and b[5] > self.cbObj.thresh:
                class_name = self.classnames[self.model.label_offset[int(b[4])]]
                if self.cbObj.target_class in class_name:
                    box = [int(b[0] * img.shape[1]),\
                           int(b[1] * img.shape[0]),\
                           int(b[2] * img.shape[1]),\
                           int(b[3] * img.shape[0])]
                    img = self.overlay_bounding_box(img, box, class_name)
                    reset = False
                    break

        self.cbObj(img, box, class_name, reset)

        if self.debug:
            self.debug.log(self.debug_str)
            self.debug_str = ""

        return img

    def overlay_bounding_box(self, frame, box, class_name):
        """
        draw bounding box at given co-ordinates.

        Args:
            frame (numpy array): Input image where the overlay should be drawn
            bbox : Bounding box co-ordinates in format [X1 Y1 X2 Y2]
            class_name : Name of the class to overlay
        """
        box_color = (20, 220, 20)
        text_color = (240, 240, 240)
        cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), box_color, 2)
        cv2.rectangle(frame, (int((box[2] + box[0])/2) - 5, \
        int((box[3] + box[1])/2) + 5), (int((box[2] + box[0])/2) + 160,\
        int((box[3] + box[1])/2) - 15), box_color, -1)
        cv2.putText(frame, class_name, (int((box[2] + box[0])/2), \
        int((box[3] + box[1])/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, \
        text_color)

        if self.debug:
            self.debug_str += class_name
            self.debug_str += str(box) + '\n'

        return frame
