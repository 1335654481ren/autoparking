
--------------------------------------------------------------------------------------

iOS 地图 SDK V4.2.0是适用于iOS系统移动设备的矢量地图开发包

--------------------------------------------------------------------------------------

地图SDK功能介绍（全功能开发包）：

地图：提供地图展示和地图操作功能；

POI检索：支持周边检索、区域检索和城市内兴趣点检索；

地理编码：提供经纬度和地址信息相互转化的功能接口；

线路规划：支持公交、驾车、步行三种方式的线路规划；

覆盖物图层：支持在地图上添加覆盖物（标注、几何图形、热力图、地形图图层等），展示更丰富的LBS信息；

定位：获取当前位置信息，并在地图上展示（支持普通、跟随、罗盘三种模式）；

离线地图：使用离线地图可节省用户流量，提供更好的地图展示效果；

调启百度地图：利用SDK接口，直接在本地打开百度地图客户端或WebApp，实现地图功能；

LBS云检索：支持查询存储在LBS云内的自有数据；

特色功能：提供短串分享、Place详情检索、热力图等特色功能，帮助开发者搭建功能更加强大的应用；


--------------------------------------------------------------------------------------
 
 
 【 新 版 提 示 】
 【 注 意 】
 1、自V3.2.0起，百度地图iOS SDK全面支持HTTPS，需要广大开发者导入第三方openssl静态库：libssl.a和libcrypto.a（存放于thirdlib目录下）
 添加方法：在 TARGETS->Build Phases-> Link Binary With Libaries中点击“+”按钮，在弹出的窗口中点击“Add Other”按钮，选择libssl.a和libcrypto.a添加到工程中 。
 
 2、支持CocoaPods导入
 pod setup //更新CocoPods的本地库
 pod search BaiduMapKit  //查看最新地图SDK


V4.2.0版本：
【新 增】
1.新增步骑行组件BaiduMapAPI_WalkNavi，WalkNavi组件需要和Base组件，Map_For_WalkNavi组件，Search组件，Location组件，Utils组件配合使用。其中需要注意的是，如果使用步骑行WalkNavi组件，则相应的地图功能需要使用Map_For_WalkNavi组件，而不是Map组件。
2.POI检索（城市检索，周边检索）新增加父子节点功能 。当scope=2时，Poi的详细信息字段（BMKPOIDetailInfo）下新增children <BMKPOIDetailChildrenInfo>字段
3.Sug检索 新增加父子节点功能（该功能需要权限）。当scope=2时，Suggestion检索结果（BMKSuggestionInfo）下新增children <BMKSuggestionChildrenInfo>字段
4.GC检索的返回结果BMKGeoCodeSearchResult中，新增precise, confidence, level等字段。
5.RGC检索的返回结果BMKReverseGeoCodeSearchResult中，新增poiRegions字段

【优 化】
1.不再区分普通版和Bitcode版，只发布支持Bitcode的版本，如果不需要Bitcode功能，可以自行剥离。以Base组件为例：
xcrun bitcode_strip -r BaiduMapAPI_Base -o BaiduMapAPI_Base
2.考虑到armv7兼容armv7s，因此不再提供armv7s的CPU架构。
3.Map组件的体积缩减了13%。
4.优化了地图引擎的内存管理
5.不再提供Radar周边雷达组件
6.不再提供Location定位组件，开发者可以使用定位SDK实现定位功能。

【修 复】
1.若干bug修复

------------------------------------------------------------------------------------------------
