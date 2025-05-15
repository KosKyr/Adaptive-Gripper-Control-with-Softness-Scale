from pipeline import Pipeline

if __name__ == "__main__":
    pipe = Pipeline(
        label_color=(0, 255, 0),
        box_color=(0, 0, 255),
        mask_color=(255, 0, 0),
        alpha=0.5,
        pad=0,
        confidence_threeshold=0.5,
        yolo_weights_path="yolo-weights/yolov8n.pt",
        dataset_path="dataset/",
        use_data_set=False,
        filter_classes=["cell phone"]
    )
    pipe.main()
