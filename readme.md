
# 1.配置python环境
cd /home/kenan/learn/autoRecord/sim/logtool

## 创建python虚拟环境
sudo apt install python3-venv
python3 -m venv .venv
source .venv/bin/activate

## 默认，很慢
python -m pip install -U pip
python -m pip install "numpy>=1.22.4,<2" "pandas<3" "matplotlib<4"
## 阿里云
python -m pip install -U pip -i https://mirrors.aliyun.com/pypi/simple --trusted-host mirrors.aliyun.com
python -m pip install "numpy>=1.22.4,<2" "pandas<3" "matplotlib<4" -i https://pypi.tuna.tsinghua.edu.cn/simple --trusted-host pypi.tuna.tsinghua.edu.cn

# 2. 编译
cmake -S . -B build
cmake --build build -j

# 3. 执行
## 依据图片生成，地图和轨迹，并且生成假的清扫数据
./build/logtool --from_floorplan --img data/floorplan.jpg --duration 120 --out data/fake_cleaning.log
## 可实话生成的地图和轨迹
python3 tools/visualize_assets.py --grid assets/grid_map.npy --traj assets/trajectory.csv --res 0.05

## 检查输出日志
./build/logdump --log data/fake_cleaning.log --limit 50
./build/logdump --log data/fake_cleaning.log --type 101 --limit 20 --parse
./build/logdump --log data/fake_cleaning.log --type 102 --limit 5 --hex 64

## 运行log_web_viewer
./build/log_web_viewer --log data/fake_cleaning.log --web ./web --port 8080

## 浏览器查看
http://127.0.0.1:8080/
