# Review：把 concatenate node 的核心改為 ROS-runtime-free 並加上 Python binding

> 這份改動沿用 distortion corrector 系列(見 `README_01`~`README_03`)的同一套作法,
> 套用到 `concatenate_data`(點雲時間同步 / 串接)子系統:把**合併核心**從 ROS
> runtime 抽離,讓它能在離線、純 Python、無 ROS node 的情境下執行,同時**原本的
> universe node 行為不變**。

---

## 1. 一句話總結

把 `CombineCloudHandler`(點雲串接的核心:座標轉換 → 運動補償 → 串接 → 產生
`ConcatenatedPointCloudInfo`)重構成 **「純邏輯 + 依賴注入」**:核心不再依賴 **ROS
執行期**(rclcpp node、`tf2_ros` 查 TF、wall clock)與 `pcl_ros`;外參(sensor→output
frame)由呼叫端注入;原本要 log 的狀況改以回傳值回報。TF 查詢、logging、訂閱/發佈、
collector 計時仍由 node 負責。核心以**點雲訊息型別**(`sensor_msgs::msg::PointCloud2`,
而非帶 `rclcpp::Publisher/Subscription` 的 trait)做模板,因此 `libconcatenate_core.so`
**完全不連結 rclcpp / tf2 / pcl**(`ldd` 僅見 libstdc++/libc),與 distortion corrector
core 同級為真正的零-rclcpp `.so`(達成方式見 §2A)。並透過 pybind
(CDR bytes 跨界)把核心暴露給 Python,額外提供一個離線、以時間戳驅動的 `Concatenator`。

最終目的:離線時不依靠 ROS,只用 Python 就能 concat 點雲(對應
`Concatenator(tf_static, params)` + `process_cloud(topic, cloud)`)。

## 2. 變更內容(依層次)

### A. 核心改為 node-free / TF-free / clock-free **且零-rclcpp**(`combine_cloud_handler_base.{hpp,cpp}` / `combine_cloud_handler.{hpp,cpp}`)

> **設計重點**:核心不再持有 ROS node、不查 TF、不碰 wall clock,是確定性的、可離線執行,
> **且** `libconcatenate_core.so` 與 distortion corrector core 同級——完全不連結 rclcpp / tf2 /
> pcl(`ldd` 僅見 libstdc++/libc)。達成方式:把**合併核心**(`CombineCloudHandler` /
> `ConcatenatedCloudResult`)改成以**點雲訊息型別**(`sensor_msgs::msg::PointCloud2`)做模板,
> **而非**以帶 `rclcpp::Publisher/Subscription` 的 `PointCloud2Traits` 做模板。如此核心標頭只
> `#include <sensor_msgs/msg/point_cloud2.hpp>`,永遠看不到 trait 的 rclcpp pub/sub 型別。node /
> collector 仍以 `MsgTraits` 做模板(它們本來就要 pub/sub),內部改用
> `CombineCloudHandler<typename MsgTraits::PointCloudMessage>` 銜接核心。

**移除的 ROS runtime 依賴(node / TF / clock,屬執行期耦合)**
- `rclcpp::Node & node_`(只用於 logger + 一個 `get_parameter`)→ 改在建構子傳入
  `matching_strategy_name` 字串。
- `managed_transform_buffer::ManagedTransformBuffer`(自己查 /tf)→ 改由外界注入。
- `rclcpp::Time` / `rclcpp::Duration`(twist queue、stamp、`RclcppTimeHash`)→ 改用
  `utils::to_nanoseconds / to_seconds`,memo key 改成 `int64_t` 奈秒。
- `pcl_ros::transformPointCloud` / `pcl::concatenatePointCloud`(其標頭會間接拉進
  `tf2_ros` / `rclcpp`)→ 改用自寫的 ROS-free 幾何函式。

**新增 / 改變的介面**
- `set_transform(const geometry_msgs::msg::TransformStamped &)`:注入 sensor→output
  外參(`header.frame_id` = output、`child_frame_id` = sensor),以 child frame 為 key
  存成 `Eigen::Matrix4f`。
- `get_transform_to_output_frame(frame)`:cloud 已在 output frame → 回 identity;有
  注入 → 回該矩陣;否則回 `nullopt`(呼叫端處理)。
- `struct MotionCompensationStatus { no_twist_available; twist_time_gap_too_large; }`,
  併入 `ConcatenatedCloudResult`;`compute_transform_to_adjust_for_old_timestamp`
  改吃 `builtin_interfaces::msg::Time` 並可選地輸出 status(取代原本的 `RCLCPP_WARN`)。
- `transform_pointcloud(matrix, in, out)`:複製整朵雲後對 x/y/z 套用 4x4(其餘欄位
  原樣保留),等價於 `pcl_ros::transformPointCloud` 的矩陣版。
- `append_pointcloud(src, dst)`:同為 PointXYZIRC 佈局,直接 append point data,取代
  `pcl::concatenatePointCloud`。
- 找不到外參時:該來源標記 `SourcePointCloudInfo::STATUS_INVALID` 並跳過(不靜默放錯
  位置)。

**關於 `traits.hpp`(重要,曾走過一段彎路)**:`PointCloud2Traits` 仍同時帶
`PointCloudMessage` + `rclcpp::Publisher/Subscription`,**維持原樣不變**。我一度想把它瘦身成
「只剩 `PointCloudMessage`、node 自行推導 `rclcpp::Publisher<Msg>`」以追求核心零-rclcpp,但這會
**破壞 `autoware_cuda_pointcloud_preprocessor`**——它用同一套模板,但 `CudaPointCloud2Traits` 的
publisher 是 `cuda_blackboard::CudaBlackboardPublisher`(非 `rclcpp::Publisher`),只能由 trait 帶著。
**正解不是動 trait,而是把核心改成以訊息型別(非 trait)做模板**(見上):pub/sub 型別仍留在
trait 裡給 node / collector / CUDA 用,但核心根本不再 include trait,因而自然零-rclcpp,兩邊都不犧牲。
唯一保留的小改動是 `cloud_collector.hpp` 直接 `#include <rclcpp/rclcpp.hpp>`(它直接用到
`rclcpp::TimerBase`,不該靠 traits 間接取得)。

### B. Node(維持運作,承接 ROS 黏合)
- node 自己持有 `ManagedTransformBuffer`;在 `cloud_callback` 中對非 output frame 的
  雲查 `output ← sensor` 並 `set_transform` 注入(靜態外參,buffer 會快取)。
- `publish_clouds` 讀 `motion_compensation_status`,印出與原本一模一樣的兩則警告。
- 建構時把 `params_.matching_strategy` 傳給核心(取代核心讀 node parameter)。

### C. Python binding
- `concatenate_pointclouds_pybind.cpp`:`CombineCloudHandler` 包裝,訊息以 CDR bytes
  進出;只有 pybind 層連 `rclcpp`(序列化用)。回傳 `(concat_bytes, info_bytes,
  transformed_map|None, MotionCompensationStatus, topic→original_stamp)`。
- `python/autoware_pointcloud_preprocessor/concatenate_pointclouds.py`:
  - `CombineCloudHandler` — 薄包裝(rclpy 訊息 ↔ bytes),一次合併一組雲。
  - **`Concatenator`** — 有狀態、以**時間戳**(非 ROS timer)驅動的離線收集器,
    **同時支援 `naive` 與 `advanced` 兩種匹配策略**,匹配本身委派給**與 node 共用的 ROS-free
    核心**(`matching_policy.hpp/.cpp`,透過 pybind 呼叫,不再有 Python 移植版):
    `process_cloud(topic, cloud) -> List[ConcatenatedFrame]`,每個 frame 的
    `status` 為 `complete`(集滿所有 input topic)或 `timeout`(資料時鐘前進超過
    `timeout_sec` 而提前關閉);還沒湊滿時回空 list。重活仍由 C++ 核心精確完成。
    - `advanced` 需提供 `lidar_timestamp_offsets` / `lidar_timestamp_noise_window`(各一個
      per input topic,單位秒,順序對齊 `input_topics`)—— 與 node 從 parameter 讀的值相同。
      匹配規則完全照 C++:某朵雲 `stamp - offset[topic]` 落在某 collector 的
      `[ref ± noise_window]`(再各自放寬 incoming topic 的 noise)即歸入該組;新 collector 的
      參考時間 = `stamp - offset[topic]`、noise = `noise[topic]`。
    - **逾時改為資料驅動**:node 用 per-collector 的 wall-clock timer,離線版改為「當資料時鐘
      (offset 修正後的時間戳)前進超過某 collector 的參考時間 `timeout_sec` 時關閉它」,因此
      離線確定性;呼叫端應**依時間戳順序**餵入。可同時維護多個 collector(在 timeout 視窗內),
      由資料驅動的逾時自然收斂;串流結束時呼叫 `flush()` 收尾。

### D. CUDA 套件 lockstep 重構 + GPU runtime 測試(`autoware_cuda_pointcloud_preprocessor`)
`autoware_cuda_pointcloud_preprocessor` 透過同一套模板重用了 `CombineCloudHandlerBase` / 模板節點。
核心介面改變後它**必須同步改**(否則 universe 建置會壞,見第 6 節 Review 第 0 點):
- 特化從 `CombineCloudHandler<CudaPointCloud2Traits>` 改為
  **`CombineCloudHandler<cuda_blackboard::CudaPointCloud2>`**(對齊核心「以訊息型別做模板」),
  並改用新 base 建構子(去掉 node、加 `matching_strategy_name`)。
- 兩處 `managed_tf_buffer_->getTransform<Eigen::Matrix4f>(...)` 改用 base 注入後的
  `get_transform_to_output_frame(frame)`(keep-input-frame 路徑取其 `.inverse()`),`node_` 不再使用。
- **修掉一個潛在 latent bug**:keep-input-frame 路徑的 `Eigen::Matrix4f::inverse()` 之前只 include
  `<Eigen/Core>`(只有宣告、無定義),`.so` 因 shared-lib 容許未定義符號而**照樣建置成功**,卻在
  `libcuda_pointcloud_preprocessor_lib.so` 留下未定義的 `MatrixBase::inverse()`。是**寫 GPU 測試
  時連結階段才抓到**的。已在 `cuda_combine_cloud_handler.cpp` 與核心 `combine_cloud_handler.cpp`
  補上 `#include <Eigen/Dense>`(`nm -DC` 確認該未定義符號消失)。
- **新增 GPU runtime gtest**(`test/cuda_concatenate_data/test_cuda_combine_cloud_handler.cpp`):在
  device memory 建兩朵雲 → `combine_pointclouds` → 拷回 host → 驗證 4 點齊全;此包**首個功能性測試**
  (先前僅 lint)。`SetUp()` 在無 CUDA 裝置時 `GTEST_SKIP`。連結上需用 `-Wl,--no-as-needed` 強制
  保留 `pointcloud_preprocessor_filter_base`,以滿足 `concatenate_data` 對 `utils::is_data_layout_*`
  的懸空未定義符號(production node 因直接用 `Filter` 基底而本來就會帶上,測試不會,故須明確指定)。

### E. 建置 / 測試(`CMakeLists.txt`)
- 新增 `concatenate_core`(SHARED;只含 combine_cloud_handler + base + matching_policy 三檔,
  **不含** node / 舊版 node,避免拖入其 `utils::is_data_layout_*` 相依)+ `_concatenate_pointclouds_pybind`
  (連 `concatenate_core` + `rclcpp`)。`concatenate_core` **不**用 `ament_target_dependencies`(那會把
  整個套件 closure、含 rclcpp 拉進來),改以 `target_link_libraries(... PUBLIC ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${autoware_sensing_msgs_TARGETS} ${builtin_interfaces_TARGETS})`
  只連訊息型別、include dir 走 header-only(autoware_utils / autoware_point_types /
  point_cloud_msg_wrapper / tf2 的 LinearMath)。結果 `libconcatenate_core.so` 的 NEEDED 只剩
  libstdc++/libm/libgcc/libc——零 rclcpp。只有 pybind 層連 `${rclcpp_LIBRARIES}`(序列化用)。
- `package.xml` 無需新增依賴(nav_msgs / autoware_sensing_msgs / point_cloud_msg_wrapper 皆已是依賴)。
- 既有 C++ 單元測試改為**注入 transform**(移除 `tf2_ros::StaticTransformBroadcaster`
  + spin)、改新建構子簽名、structured binding 補第 5 個元素。新增 pytest
  `test_concatenate_pointclouds_py`。
- **(CUDA 套件 `CMakeLists.txt` / `package.xml`)** 新增 `test_cuda_combine_cloud_handler`
  GPU gtest(`ament_add_gtest` + 連 `cuda_pointcloud_preprocessor_lib` + 強制保留 filter_base,
  見 §2D)、`package.xml` 補 `<test_depend>ament_cmake_gtest</test_depend>`。

## 3. 檔案清單

**修改**
- `include/.../concatenate_data/combine_cloud_handler_base.hpp` / `src/.../combine_cloud_handler_base.cpp`
  (`ConcatenatedCloudResult` 改以 `PointCloudMsgT` 做模板)
- `include/.../concatenate_data/combine_cloud_handler.hpp` / `src/.../combine_cloud_handler.cpp`
  (改成 `CombineCloudHandler<sensor_msgs::msg::PointCloud2>` 特化;drop `traits.hpp`、改 include
  `point_cloud2.hpp`;補 `#include <Eigen/Dense>` 修 `inverse()`)
- `include/.../concatenate_data/traits.hpp`(維持原樣;核心改為非 trait 模板後不再需要動它)
- `include/.../concatenate_data/cloud_collector.{hpp,ipp}`(補 `#include <rclcpp/rclcpp.hpp>`;改用
  `CombineCloudHandler<MsgTraits::PointCloudMessage>`)
- `include/.../utility/conversion.hpp`(新增 `to_ros_time`)
- `include/.../concatenate_data/concatenate_and_time_sync_node.hpp` / `.ipp`(node;改用訊息型別模板的核心)
- `include/.../concatenate_data/collector_matcher.hpp` / `.ipp`(改為委派給共用核心)
- `test/test_concatenate_node_unit.cpp`
- `CMakeLists.txt`
- **(CUDA 套件)** `autoware_cuda_pointcloud_preprocessor` 的 `cuda_combine_cloud_handler.{hpp,cpp}`
  (lockstep:特化於 `cuda_blackboard::CudaPointCloud2`、新 base 建構子 + `get_transform_to_output_frame`、
  補 `#include <Eigen/Dense>`)、`CMakeLists.txt`(註冊 GPU 測試)、`package.xml`(`ament_cmake_gtest`)

**新增**
- `include/.../concatenate_data/matching_policy.hpp` / `src/.../matching_policy.cpp`(ROS-free 匹配核心,node 與離線共用)
- `src/.../concatenate_data/concatenate_pointclouds_pybind.cpp`
- `python/autoware_pointcloud_preprocessor/concatenate_pointclouds.py`
- `test/test_matching_policy.cpp`(匹配核心的 gtest)
- `test/test_concatenate_pointclouds_py.py`
- **(CUDA 套件)** `test/cuda_concatenate_data/test_cuda_combine_cloud_handler.cpp`(GPU runtime gtest)

## 4. 驗證結果
- `colcon build`(**`autoware_pointcloud_preprocessor` + `autoware_cuda_pointcloud_preprocessor` 兩包**)
  皆通過 —— CUDA 套件在 lockstep 重構後重新建置成功(原本被核心介面變更弄壞)。
- **零-rclcpp 已驗證**:`ldd libconcatenate_core.so` 無任何 rclcpp / tf2 / pcl;`objdump -p` 的
  NEEDED 只有 libstdc++ / libm / libgcc / libc(訊息型別 header-only,連 sensor_msgs `.so` 都未列入)。
  `_concatenate_pointclouds_pybind` 則如預期連 `librclcpp.so`(序列化用)。
- `test_concatenate_node_unit`(C++,8 個:合併 / 運動補償,改注入 transform)→ Passed
  (node 的匹配改委派給共用核心後行為不變)。
- `test_matching_policy`(C++ gtest,8 個,直接測匹配核心:naive 最近匹配 / 跳過已有
  topic、advanced 視窗內外、ctor 尺寸驗證、advanced 不看 has_topic)→ Passed。
- `test_concatenation_info`(17)、`test_distortion_corrector_node`(24)、`test_conversion`(4)、
  `test_utilities`(4)→ Passed(確認 message-type 重構與 `<Eigen/Dense>` 補丁無回歸)。
- `test_concatenate_pointclouds_py`(pytest,**13 個**:合併三朵雲、原始時間戳、naive
  complete/timeout、advanced 需要 offsets、advanced 依 offset 分組、advanced 逾時、advanced
  不併入 window 外的雲;**新增**:非單位外參(旋轉+平移)數值驗證、keep-input-frame 的
  `inverse()` 來回、帶 twist 的運動補償位移、缺外參時丟棄並回報該 frame、未知 topic 應拋
  `ValueError`)→ Passed(Python 端的匹配走的就是綁定的共用核心)。
  `test_distortion_corrector_py`(4)→ Passed。
- **`test_cuda_combine_cloud_handler`(GPU runtime gtest,2 個)→ Passed on NVIDIA RTX 3060 Laptop
  GPU(driver 580.159.03)**:`ConcatenatesTwoCloudsOnGpu`(device memory 建兩朵雲、
  `combine_pointclouds`、拷回 host、驗證 4 點齊全)與 `DropsCloudWithoutExtrinsic`(缺外參的雲被
  丟棄、`dropped_frames_missing_transform` 回報該 frame)。此包**首個功能性測試**,端到端證明 CUDA
  handler 的 lockstep 重構在真實 GPU 上正確。
- 總計:**65 個 C++ gtest + 13 個 pytest + 2 個 GPU gtest 全綠**。非單位外參與運動補償路徑現在已有
  數值測試(先前只走 identity);兩個 handler 的「缺外參」行為已對齊為丟棄 + STATUS_INVALID + 經由
  結果回報(node 印 throttled 警告)。

## 5. 使用方式(離線 Python)

```python
from autoware_pointcloud_preprocessor.concatenate_pointclouds import Concatenator

concatenator = Concatenator(
    input_topics=["lidar_top", "lidar_left", "lidar_right"],
    output_frame="base_link",
    tf_static=tf_static,        # {sensor_frame: TransformStamped(header=base_link, child=sensor)}
    timeout_sec=0.2,
    is_motion_compensated=True,
    matching_strategy="advanced",                 # 與 universe node 同一套匹配規則
    lidar_timestamp_offsets=[0.0, 0.04, 0.08],    # per input topic(秒)
    lidar_timestamp_noise_window=[0.01, 0.01, 0.01],
)
concatenator.process_twist(twist_msg)             # 餵 twist / odom 供運動補償(依時間戳順序)

# process_cloud 回傳一個 list(可能 0、1 或多個 frame)
# arrival_time(秒,必填)= 該雲的「接收時間」,例如 rosbag 的 record timestamp(messages.timestamp)。
# timeout 與 naive 匹配都由到達時間驅動,忠實重現線上 node 的 wall-clock timer;請「依到達順序」餵入(見 §7)。
for frame in concatenator.process_cloud(topic, undistorted_cloud, arrival_time=bag_recv_sec):
    if frame.status == "complete" or (frame.status == "timeout" and allow_frame_drop):
        concat_pc2 = frame.result.concatenated_cloud
        lidar_info = frame.result.concatenation_info

# 串流結束時把還沒關閉的群組收尾
for frame in concatenator.flush():
    ...
```

---

## 6. Review 與建議

### 整體評價

✅ **沿用 distortion corrector 的解耦模式,且達到同級的零-rclcpp**:核心邏輯不再持有 node、不查
TF、不碰 wall clock,是確定性的、可離線執行;`libconcatenate_core.so` 不連結 rclcpp / tf2 / pcl
(`ldd` 已驗證);node 行為由既有單元測試把關不變;Python 端以 CDR bytes 跨界、可重現;CUDA 路徑
已有 GPU runtime 測試實證。沒有發現破壞數值正確性的問題。

### 值得注意 / 建議事項

0. **教訓:核心介面是被 CUDA 套件重用的共享 base —— 改它會牽動 `autoware_cuda_pointcloud_preprocessor`。**
   我一開始只單獨建 `autoware_pointcloud_preprocessor` 就宣稱完成,漏看 CUDA 套件透過同一套模板重用
   `CombineCloudHandlerBase` / 模板節點。改了 base 後 **CUDA 套件建置直接壞掉**(`CUDA_BUILD_EXIT=2`:
   舊 `CombineCloudHandlerBase(node,…)` 建構子不見了、`managed_tf_buffer_`/`node_` 被移除)。已透過
   lockstep 重構 CUDA handler 修好,並**同時建置兩包**驗證。**建議**:動到 `sensing/` 下被跨套件重用的
   共享標頭時,務必把所有 consumer(尤其 CUDA)一起 build。

   **延伸教訓:lockstep 改完「能建置」不等於「正確」。** lockstep 時我在 keep-input-frame 路徑寫了
   `Eigen::Matrix4f::inverse()`,但只 include 了 `<Eigen/Core>`(只有宣告、定義在 `<Eigen/LU>`)。
   因為 shared library 預設容許未定義符號,`libcuda_pointcloud_preprocessor_lib.so` **照樣建置成功**,
   只是埋了一個未定義的 `MatrixBase::inverse()`——production node 因把它連進元件 `.so` 而僥倖在載入時
   解析,但這是不該存在的 latent 風險。**是寫 GPU 測試、在測試的連結階段才把它逼出來的**(`nm -DC`
   實證)。已補 `<Eigen/Dense>` 修好。這直接印證了「寫 test」的價值:純 build 通過會漏掉 shared-lib
   的未定義符號。

1. **匹配策略已對齊 `advanced`,且為單一實作(無 drift 風險)。**
   `naive` / `advanced` 的匹配邏輯已抽成 **ROS-free 的共用核心**
   (`matching_policy.hpp/.cpp`:`NaiveMatchingPolicy` / `AdvancedMatchingPolicy`,
   以 plain-data 的 `IncomingCloudInfo` / `CandidateCollectorState` / `CollectorReference` 為介面)。
   - **node 端**:原本的 `NaiveCollectorMatcher<MsgTraits>` / `AdvancedCollectorMatcher<MsgTraits>`
     改為薄轉接層 —— 把 `CloudCollector` 的 `CollectorInfo` 讀成核心的 plain view、委派核心做決策、
     再把結果映回 collector(對外 API 不變,既有測試證明行為不變)。
   - **離線 Python 端**:`Concatenator` 透過 pybind 呼叫**同一個核心**,不再有 Python 移植版。
   - 因此線上/離線共用唯一一份匹配實作,`test_matching_policy`(gtest)直接鎖定其行為。
   - **唯一刻意保留的行為差異**:逾時從 node 的 **wall-clock timer** 改為**資料驅動**(離線以
     時間戳判定 collector 是否已不可能再收到雲;用核心的 `reference_for()` 取得 offset 修正後的
     「現在時間」)。這是離線確定性的必要設計,且**不在匹配核心內**(屬 Concatenator 的收集層);
     代價是呼叫端須**依時間戳順序**餵入,docstring 已標明。
   - **後續強化**:`process_cloud` 現在可選地吃 `arrival_time`(rosbag 的 `messages.timestamp`),
     讓 timeout 改由**到達時間**驅動而非 header stamp,更忠實重現 node 的 wall-clock timer;不給
     `arrival_time` 時退回上述 stamp 驅動行為。選擇方式與實測(advanced+arrival 重現 golden、
     naive+arrival 會漂)見 **§7**。

2. **找不到外參時的行為(已對齊兩個 handler + 加上警告)。**
   舊版透過 `managed_tf_buffer_->transformPointcloud` 查 TF;新版要求**事先注入**,缺
   外參時把該來源標 `STATUS_INVALID` 並跳過。
   - 兩個 handler 已**對齊**:PointCloud2 與 CUDA 皆為「丟棄該雲 + STATUS_INVALID」(先前 CUDA 是
     退回 identity、會把點靜默放錯位置;CUDA 第二趟 sync-publish 也同步跳過,維持 `output_points`
     讀取位移對齊)。
   - **不再靜默**:被丟棄的 frame 經由結果的 `dropped_frames_missing_transform` 回報;共用的
     `publish_clouds` 對它印一則 throttled 警告(同時涵蓋 PointCloud2 與 CUDA node),離線 Python 端
     也能在 `ConcatenationResult` 讀到。
   - **已補測試**:pytest `test_missing_transform_drops_frame_and_reports_it`、GPU
     `DropsCloudWithoutExtrinsic`。(node 端「TF 查不到」這條 ROS 路徑仍建議補一個 node-level 測試。)

3. **非單位外參與運動補償的數值測試(已補)。**
   先前 C++ 單元測試與 pytest 用的雲都在 `base_link`(= output frame),`get_transform_to_output_frame`
   一律回 identity,**真正的非單位外參路徑沒有被數值驗證**。現已補上 pytest:
   - `test_non_identity_transform_is_applied`:注入 90° 旋轉 + 平移,驗證輸出點座標(覆蓋
     `to_eigen_matrix` 的四元數→矩陣與 `transform_pointcloud`)。
   - `test_keep_input_frame_returns_points_in_sensor_frame`:驗證 keep-input-frame 的 `inverse()`
     來回會還原原始 sensor-frame 座標。
   - `test_motion_compensation_shifts_newer_cloud`:餵入定速 twist,驗證較新的雲被位移
     0.1 s × 1.0 m/s = 0.1 m(覆蓋 `correct_pointcloud_motion` + `compute_transform_to_adjust_*`)。

4. **`transform_pointcloud` 的效能。**
   先 `out = in`(整朵雲深拷貝)再逐點 `4x4 * (x,y,z,1)`。對大點雲是一次完整拷貝 + 無向量化
   的逐點乘法。對離線批次而言可接受(`pcl_ros` 也差不多),但若日後有效能需求,可考慮就地
   轉換或 SIMD/批次化。屬備註,非阻擋。

5. **`append_pointcloud` 假設兩朵雲佈局完全相同。**
   目前所有輸入都先過 `convert_to_xyzirc_cloud` 統一成 PointXYZIRC,所以 `point_step` 一致、
   直接 append bytes 是對的。但這是個隱性前提:萬一日後有人在未轉換的雲上呼叫它,會產生
   錯位資料。**建議**加一個 `assert(src.point_step == dst.point_step)`(或 fields 檢查)當
   護欄。

6. **核心 `.cpp` 被編譯兩次**(`concatenate_data` 元件庫 + 較小的 `concatenate_core`)。兩者連結性質
   **不同**:`concatenate_data` 是給 node 用的、會連 rclcpp/pcl;`concatenate_core` 只含 3 個檔案
   (不含 node / 舊版 node)、以訊息型別做模板,**完全不連 rclcpp**(見 §2A、§4)。好處是 pybind 既不
   拖入 `utils::is_data_layout_*` 等 node 相依,也得到一個零-rclcpp 的核心 `.so`;代價是建置時間/體積
   略增。刻意為之。

7. **序列化的每次呼叫成本。**
   每次 `combine_pointclouds` 都對每朵輸入雲與輸出雲做 CDR 序列化/反序列化。對離線、求正確
   與可重現的定位可接受;若要處理超大量 frame,可考慮日後提供 numpy buffer 零拷貝路徑(點雲
   走 buffer,transform/twist 仍走序列化)。屬備註。

### 結論

架構方向與實作品質良好,`autoware_pointcloud_preprocessor` 與 `autoware_cuda_pointcloud_preprocessor`
**兩包皆建置通過**、測試皆綠;核心邏輯 node-free / TF-free / clock-free 且確定性,**`.so` 已驗證
為零-rclcpp**(與 distortion core 同級);`naive`/`advanced` 兩種匹配已抽成**單一共用核心**(node
與離線 Python 都走它,無 drift 風險)並有 gtest + pytest 覆蓋;CUDA 路徑已有 **GPU runtime 測試實證**
(RTX 3060,並藉此抓出並修掉一個 latent `inverse()` 未定義符號)。**非單位外參 + 運動補償**路徑現在
也有數值測試(第 3 點),缺外參的丟棄行為已對齊兩個 handler 並改為**回報 + 警告**(第 2 點)。可作為
「離線 Python concat」的穩固基礎。合併前/後仍可追補的是:
- **node 端 TF 缺失路徑的 ROS-level 測試**(第 2 點;handler 層的丟棄/回報已有 pytest 與 GPU 測試,
  但 node 在 `cloud_callback` 查 TF 失敗 → 不注入這條路徑尚無整合測試);
- **非單位外參的 GPU 數值測試**(目前 GPU 只走 identity 與丟棄;CPU/pybind 已涵蓋旋轉+平移)。

---

## 7. 離線輸出與線上 golden 的數值驗證(matching / timeout 模擬真實情形)

> 這節記錄「如何驗證離線 `Concatenator` 的輸出 == 線上 node 的實際輸出」,以及如何選擇 **匹配策略**
> 與 **`timeout_sec`** 才能既重現 golden、又忠實模擬 real-time 的到達/逾時行為。

### 7.1 離線的 timeout 由「到達時間」驅動

線上 node 的 per-collector timeout 是 **wall-clock timer**,從某 collector 收到第一朵雲(**到達時間**)起算,
`timeout_sec` 後關閉。離線版的逾時是 `Concatenator` 在 Python 端自己做的(不在共用的 matching 核心內),
**以每朵雲的 `arrival_time` 為時鐘**——即 rosbag 的 record timestamp(`messages.timestamp`,接收時間),
wall-clock 到達的離線類比,**忠實重現 node 的 timer 行為**。呼叫端須**依到達順序**餵入並傳 `arrival_time`。

> header stamp 仍會用到,但**只用於匹配**(advanced 的 `stamp − offset`)與**運動補償**,**不**當 timeout
> 時鐘。(早期曾有一個以 header stamp 驅動 timeout 的 fallback,已移除——離線只剩 arrival 一種時鐘。)

### 7.2 golden 來源與比對方法

`rosbag2_before_concat` 同時錄了 **8 顆 lidar 的 `*_before_sync` 輸入** 與 **node 自己的輸出**
`/sensing/lidar/concatenated/pointcloud`(= golden,共 N frame)。比對流程(`offline_concat.py` 直讀
bag 跑離線 concat;golden 的逐 frame 數值比對是一支小腳本,同樣直讀 bag 的兩個 topic):

1. 用 node param(`concatenate_and_time_sync_node.param.yaml`)的 `input_topics` **順序**、`lidar_timestamp_offsets`、
   `lidar_timestamp_noise_window`、`timeout_sec` 建 `Concatenator`(advanced 的 offsets 是 positional,順序必須一致)。
2. 把 twist + 8 路 lidar 依**接收順序**餵入,並把每朵雲的 `messages.timestamp` 當 `arrival_time` 傳進去
   (見 §7.6)。
3. 每個 `COMPLETE` frame 依輸出 header stamp **配對最近的 golden**;比 **點數(Δpts)** 與
   **bbox / 形心座標**(對點順序不敏感)。

### 7.3 重現指令

```bash
# 用 ROS python(非 conda),build + source 後:
source /opt/ros/humble/setup.bash && source install/setup.bash

# 直讀 bag 跑離線 concat(觀察 complete / timeout 行為;arrival 驅動會自動帶入 bag 的 recv timestamp)。
# advanced 用 --param-file 鏡射 node:param 帶著 input_topics 順序,positional 的 offsets 不會被排錯。
python3 offline_concat.py rosbag2_before_concat \
    --param-file src/autoware/launcher/aip_launcher/aip_x2_gen2_launch/config/concatenate_and_time_sync_node.param.yaml
```

### 7.4 實測結果(rate 1 重錄的 bag;到達散布 ≈ stamp 散布 ≈ 0.12 s,真實 10 Hz;301 golden)

**策略對比(`timeout_sec=0.2` = node 值,arrival 驅動)**:每個 `COMPLETE` frame 與最近 golden 比對。

| 策略 | complete | 對到 golden | Δpts | 座標差(bbox / 形心) |
|---|---|---|---|---|
| **advanced** | 297 | **297/297** | **全 0** | 0.015 mm / 2 mm |
| naive | 297 | 297/301 | **max 1434 / mean 103** | — |

- `advanced` 的每個 complete frame 都 **point-count bit-exact、座標亞毫米**對上 golden(~2 mm 形心殘差是
  **運動補償的 twist 內插時間點**差異,非分組差異)。297 vs 301 的差距是到達抖動下被 timeout 丟掉的 4 個
  frame —— 線上 node 在 `timeout_sec=0.2` 下也會丟。
- `naive` 大幅漂移(max 1434 點):它的匹配**直接用到達時間當 key**,抖動會擾動「最近鄰」分組。

**timeout 掃描(advanced,arrival)**:調大 `timeout_sec` 會減少被丟的 frame。

| `timeout_sec` | complete | timeout |
|---|---|---|
| 0.2(node 值) | 297 | 10 |
| 0.3 | 300 | 3 |
| **0.5** | **301** | 1\* |

\*0.5 時 301 個 golden frame 全數 complete;僅剩的 1 個 timeout 是串流結束時 flush 的尾端殘缺組(最後一輪
沒收齊 8 顆就到檔尾),任何 timeout 值都無法避免。

### 7.5 結論與選擇建議

- **`advanced` + arrival 完全重現 golden**:point count bit-exact、座標亞毫米。`advanced` 的匹配用
  `stamp − offset` 當 key,**arrival 只驅動 timeout**,所以到達抖動最多讓少數 frame 提前 timeout,分組永遠
  穩定。這是「模擬 real-time 又對齊 golden」的正解。
- **`timeout_sec` 怎麼選**:要**對齊 node**(連同它在抖動下會丟的 frame 一起重現)就用 node 的值(0.2);
  要**盡量收齊所有 frame**(離線、不在意對齊 node 的丟棄行為)就**調大**(本 bag 0.5 → 301/301)。
- **`naive` 會大幅漂移(max 1434 點),是設計上的而非 bug。** `naive` 直接用到達時間當匹配 key;線上 node 的
  naive 模式在真實抖動下也一樣脆弱。優先用 `advanced`。
- **bag 的到達時間必須真實**:用 `ros2 bag record`(rate 1)在車上/即時錄製。慢速轉錄或 `bag play` 降速會讓
  `messages.timestamp` 出現大空檔(例如曾見過同一輪散布 0.44 s、5 s stall),會把每組都逾時關掉(0 complete)
  —— 那是 bag artifact,不是程式錯誤。

### 7.6 餵入順序與 timeout 的實務規則(必讀)

timeout 不是真的計時器,而是「到達時鐘有沒有前進超過 `timeout_sec`」。因此:

- **`process_cloud(topic, cloud, arrival_time=recv_sec)` 的 `arrival_time` 是必填**,沒有 header-stamp
  fallback(已移除)。
- **依接收順序餵入,不要依 header stamp 重排。** rosbag 用 `order by timestamp` 讀出來就是接收順序,直接餵
  並把 `messages.timestamp` 當 `arrival_time` 傳。亂序餵會讓到達時鐘亂跳、逾時誤判(rosbag 是按接收順序存
  的,**不等於 stamp 順序**——較早量測的雲可能較晚才被錄到)。
- **`timeout_sec` 是「一組等齊缺漏 topic 的時間」**:對齊 node 用 node 值(0.2,連同它會丟的 frame 一起
  重現);要盡量收齊就調大(見 §7.4 掃描)。`advanced` 調大安全(stamp 視窗仍框住誰能入組,只是多等、不會
  誤併);`naive` 調大配抖動可能誤併,故優先用 `advanced`。需要**到達時間真實的 bag**(rate 1 即時錄製)。

> `offline_concat.py` 已示範正確做法:按 bag 的接收順序讀出、把 `messages.timestamp` 當 `arrival_time` 傳入。

### 7.7 對應的單元測試

- `test_concatenate_pointclouds_py.py`(`timeout_sec=0.2`,皆 arrival 驅動):
  `test_arrival_time_drives_timeout_independent_of_stamp`(同 stamp、不同到達 → 由到達決定 timeout)、
  `test_arrival_time_groups_clouds_with_far_apart_stamps`(stamp 差很遠、到達很近 → 仍歸同組),
  外加 naive/advanced 的 complete / timeout / 視窗內外 / 未知 topic 等案例。
- `test_matching_policy.cpp`:直接鎖匹配核心的 `naive` 最近匹配與 `advanced` 視窗(offset 修正、noise 視窗
  內外、不看 `has_topic`),確保線上/離線共用的這份核心不漂。
