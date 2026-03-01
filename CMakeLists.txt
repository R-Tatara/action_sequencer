cmake_minimum_required(VERSION 3.8)
project(action_sequencer)

# GCC または Clang を使用している場合、厳格な警告オプションを有効化
# -Wall: 一般的な警告, -Wextra: 追加警告, -Wpedantic: 標準準拠チェック
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# 依存パッケージの検索
# find_package(<パッケージ名> REQUIRED) で依存ライブラリを取得する
# REQUIRED を付けると、見つからない場合にビルドエラーになる
# ============================================================
find_package(ament_cmake REQUIRED)                       # ROS2 CMakeビルドシステムの基盤
find_package(rclcpp REQUIRED)                            # ROS2 C++クライアントライブラリ
find_package(std_msgs REQUIRED)                          # 標準メッセージ型 (String, Int32 など)
find_package(moveit_ros_planning_interface REQUIRED)     # MoveIt2 モーションプランニングAPI

# ============================================================
# 実行ファイル（ノード）のビルド設定
# ============================================================

# add_executable(<実行ファイル名> <ソースファイル>)
add_executable(action_sequencer src/action_sequencer_node.cpp)

# インクルードディレクトリの指定
# BUILD_INTERFACE: ビルド時に参照するヘッダのパス
# INSTALL_INTERFACE: インストール後に他パッケージから参照されるときのパス
target_include_directories(action_sequencer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 使用するC/C++標準バージョンの指定
target_compile_features(action_sequencer PUBLIC c_std_99 cxx_std_17)

# ROS2依存パッケージのリンク
# find_package で見つけたパッケージのヘッダ・ライブラリを実行ファイルに紐付ける
ament_target_dependencies(
  action_sequencer
  rclcpp
  std_msgs
  moveit_ros_planning_interface
)

# ============================================================
# インストール設定
# colcon build 後に install/ フォルダへ配置するルールを定義する
# ============================================================

# 実行ファイルを lib/<パッケージ名>/ にインストール
# ${PROJECT_NAME} は project() で定義した名前に展開される → lib/action_sequencer/
install(TARGETS action_sequencer
  DESTINATION lib/${PROJECT_NAME})

# launch ディレクトリ以下のファイルを share/<パッケージ名>/launch/ にインストール
# これにより `ros2 launch action_sequencer xxx.launch.py` で起動できるようになる
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ============================================================
# テスト設定（オプション）
# colcon test 時のみ有効になるブロック
# ============================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 著作権チェックと cpplint（コードスタイルチェック）を手動で無効化
  # プロジェクト初期段階でエラーを抑制したい場合に使う設定
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# このパッケージを ament ビルドシステムに登録する（必ず最後に記述）
ament_package()