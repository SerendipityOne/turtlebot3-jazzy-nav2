import os

from yolox.exp import Exp as YoloxExp


class Exp(YoloxExp):
    def __init__(self):
        super().__init__()
        self.num_classes = 5
        self.depth = 0.33
        self.width = 0.25
        self.depthwise = True
        self.act = 'silu'
        self.input_size = (640, 640)
        self.test_size = (640, 640)
        self.random_size = (14, 26)
        self.max_epoch = 100
        self.warmup_epochs = 5
        self.no_aug_epochs = 15
        self.data_num_workers = 8
        self.eval_interval = 5
        self.data_dir = os.environ.get('TB3_COCO_DATASET', 'datasets/tb3_objects')
        self.output_dir = os.environ.get('YOLOX_OUTPUT', 'outputs/yolox_nano_tb3')
        self.train_ann = 'instances_train.json'
        self.val_ann = 'instances_val.json'
        self.exp_name = 'yolox_nano_tb3'
