# 🚦 Real World System

We are conducting research to develop a real-world traffic surveillance system.

The first benchmark named [TSBOW](ttps://skkuautolab.github.io/TSBOW/) has been accepted by the 40th AAAI Conference on Artificial Intelligence (AAAI-26). 

Please follow the [guideline](https://github.com/SKKUAutoLab/TSBOW?tab=readme-ov-file#-dataset-download) to access and download the dataset from [HuggingFace](https://huggingface.co/datasets/SKKUAutoLab/TSBOW).


<!-- ## Installation

To execute the Python source code for processing the dataset, install the required libraries in [requirements.txt](requirements.txt). -->

<!-- MARK: Datasets -->

## 📊 Benchmark Traffic Datasets

| Dataset           | Introduction                      |  Pub  | Paper |
|:---:              |:---                               | :---: | :--- |
| **UAVDT**         <br>[[website]](https://datasetninja.com/uavdt)| - *<span style="color: #FFCC00">Hardware</span>*: UAVs. <br> - *<span style="color: #33CCCC">Tasks</span>:* object detection, single object tracking, multiple-object tracking. <br> - *<span style="color: #FF6600">Position</span>:* China. <br> - *<span style="color: #6699FF">Weather</span>:* sunny/cloudy, fog, rain. <br> - *<span style="color: #FF0066">Time</span>:* day, night. | IJCV <br> 2020 | The Unmanned Aerial Vehicle Benchmark: Object Detection, Tracking and Baseline |
| **UA-DETRAC**     <br>[[website]](https://wayback.archive-it.org/org-652/20231112205116/https:/detrac-db.rit.albany.edu/)| - *<span style="color: #FFCC00">Hardware</span>*: Cannon EOS 550D camera. <br> - *<span style="color: #33CCCC">Tasks</span>:* object detection, multi-object tracking. <br> - *<span style="color: #FF6600">Position</span>:* China. <br> - *<span style="color: #6699FF">Weather</span>:* sunny/cloudy, rain. <br> - *<span style="color: #FF0066">Time</span>:* day, night. | CVIU <br> 2020 | UA-DETRAC: A new benchmark and protocol for multi-object detection and tracking |
| **AAU RainSnow**  <br>[[website]](https://vbn.aau.dk/en/datasets/aau-rainsnow-traffic-surveillance-dataset/)| - *<span style="color: #FFCC00">Hardware</span>*: RGB color and thermal camera. <br> - *<span style="color: #33CCCC">Tasks</span>:* instance segmentation, single object tracking, multiple-object tracking. <br> - *<span style="color: #FF6600">Position</span>:* Denmark. <br> - *<span style="color: #6699FF">Weather</span>:* fog, rain, snow. <br> - *<span style="color: #FF0066">Time</span>:* day, night. | ITS <br> 2019 | Rain Removal in Traffic Surveillance: Does it Matter? |
| 🎯 <br> **<span style="color: #FFCC00">T</span><span style="color: #33CCCC">S</span><span style="color: #FF6600">B</span><span style="color: #6699FF">O</span><span style="color: #FF0066">W</span>**             <br>[[website]](https://skkuautolab.github.io/TSBOW/)| - *<span style="color: #FFCC00">Hardware</span>*: CCTV system + color camera. <br> - *<span style="color: #33CCCC">Tasks</span>:* object detection. <br> - *<span style="color: #FF6600">Position</span>:* South Korea. <br> - *<span style="color: #6699FF">Weather</span>:* sunny/cloudy, haze, rain, snow. <br> - *<span style="color: #FF0066">Time</span>:* day. <br> (night-time and other tasks will be updated later) | AAAI <br> 2026 | TSBOW: Traffic Surveillance Benchmark for Occluded Vehicles Under Various Weather Conditions |

<!-- UA-DETRAC new version: (https://sites.google.com/view/daweidu/projects/ua-detrac?authuser=0) -->



<!-- MARK: Baselines -->

## 🧪 Baselines

|  Year |  Pub    | Paper                            | Link  | Note |
| :---: |  :---:  | :---                             |:---:  | :--- |
| 2024  |  ICDICI | A review on yolov8 and its advancements | [paper](https://link.springer.com/chapter/10.1007/978-981-99-7962-2_39) | YOLOv8 |
| 2024  |  arXiV  | YOLOv11: An Overview of the Key Architectural Enhancements | [paper](https://arxiv.org/abs/2410.17725) | YOLOv11 |
| 2024  |  CVPR   | DETRs Beat YOLOs on Real-time Object Detection | [paper](https://openaccess.thecvf.com/content/CVPR2024/html/Zhao_DETRs_Beat_YOLOs_on_Real-time_Object_Detection_CVPR_2024_paper.html) | RT-DETR |
| 2025  |  arXiV  | A Breakdown of the Key Architectural Features | [paper](https://arxiv.org/abs/2502.14740) | YOLOv12 |
| 2025  |  arXiV  | YOLO26: Key Architectural Enhancements and Performance Benchmarking for Real-Time Object Detection | [paper](https://arxiv.org/abs/2509.25164) | YOLO26 |

<!-- MARK: Metrics -->

## 📐 Metrics

| Metric    | Year  | Pub   | Paper | Link  | Note |
| :---:     | :---: | :---  | :---  | :---: | :--- |
| Precision | 1955 | American Documentation | Machine literature searching VIII. Operational criteria for designing information retrieval systems | [paper](https://doi.org/10.1002/asi.5090060209) | TP / (TP + FP) |
| Recall | 1955 | American Documentation | Machine literature searching VIII. Operational criteria for designing information retrieval systems | [paper](https://doi.org/10.1002/asi.5090060209) | TP / (TP + FN) |



<!-- MARK: Citation -->

## 📚 Citation 

If you find our work helpful for your research, please consider citing the following BibTeX entry.

```bibtex
@article{Huynh2026TSBOW, 
    title={TSBOW: Traffic Surveillance Benchmark for Occluded Vehicles Under Various Weather Conditions}, 
    author={Huynh, Ngoc Doan-Minh and Tran, Duong Nguyen-Ngoc and Pham, Long Hoang and Tran, Tai Huu-Phuong and Jeon, Hyung-Joon and Nguyen, Huy-Hung and Khac Vu, Duong and Jeon, Hyung-Min and Phan, Son Hong and Pham-Nam Ho, Quoc and Tran, Chi Dai and Khanh, Trinh Le Ba and Jeon, Jae Wook}, 
    journal={Proceedings of the AAAI Conference on Artificial Intelligence}, 
    volume={40}, 
    number={7}, 
    url={https://ojs.aaai.org/index.php/AAAI/article/view/37439}, 
    DOI={10.1609/aaai.v40i7.37439}, 
    year={2026}, 
    month={Mar.}, 
    pages={5239-5247} 
}
```
