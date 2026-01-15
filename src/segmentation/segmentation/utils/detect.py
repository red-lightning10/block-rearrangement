from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2 import model_zoo
from detectron2.data import MetadataCatalog
import cv2
import torch

def load_model(model_path):
    """
    Load trained model for inference
    
    Args:
        model_path: Path to trained model checkpoint
        output_dir: Output directory for configuration
        
    Returns:
        predictor: Detectron2 predictor
        metadata: Dataset metadata
    """
    
    
    # Setup config
    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = 5
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
    cfg.MODEL.WEIGHTS = model_path
    
    try:
        cfg.MODEL.DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
    except:
        cfg.MODEL.DEVICE = "cpu"
    
    predictor = DefaultPredictor(cfg)
    
    class_names = ["blue_cube", "green_cube", "orange_cube", "red_cube", "yellow_cube"]
    
    # Register metadata without loading full dataset
    MetadataCatalog.get("cubes_val").set(thing_classes=class_names)
    metadata = MetadataCatalog.get("cubes_val")
    
    return predictor, metadata

def predict_image(predictor, image_path, score_threshold=0.5):
    """
    Run inference on a single image and return predictions
    
    Args:
        predictor: Detectron2 predictor
        metadata: Dataset metadata
        image_path: Path to input image
        score_threshold: Confidence threshold for predictions
        
    Returns:
        outputs: Model predictions
    """
    image = cv2.imread(image_path)
    outputs = predictor(image)
    instances = outputs["instances"]
    keep = instances.scores >= score_threshold
    outputs["instances"] = instances[keep]
    
    return outputs