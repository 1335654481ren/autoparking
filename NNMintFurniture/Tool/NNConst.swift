//
//  NNConst.swift
//  NNMintFurniture
//
//  Created by 任晓亮 on 2018/09/27.
//  Copyright © 2018年 xiaoliang.Ren. All rights reserved.
//

import UIKit
import WebSocket
import SwiftyJSON

// 网络请求参数 offset：从第几条开始加载；limit：每次加载多少条
let NNGender = "gender"
let NNGeneration = "generation"
let NNLimit = "limit"
let NNOffset = "offset"
var gsocket: WebSocket!
var websocket_url = "http://42.62.85.20:443/ws/app"
let appUrl = "http://42.62.85.20:443/"
var tokenStr:String!
var logon:Bool = false
var gflowstage:Int = 0
var process:Bool = false
var device_id:String!
var license:String!
var car_type:String!
var route: Array<Int>!
var station_num:Int! = 0
var station_json = [JSON]()
var levelArr: Array<Any>? = ["宴会厅","酒店大堂","地下车库","地上车位"]
var stationNameArry = [Int : String]()
// 屏幕的宽
let NNScreenWidth = UIScreen.main.bounds.size.width
// 屏幕的高
let NNScreenHeight = UIScreen.main.bounds.size.height

// 顶部标题的y
let kTitlesViewY: CGFloat = 64
// 顶部标题的高度
let kTitlesViewH: CGFloat = 36
// 搜索框高度
let kSearchBarH: CGFloat = 30


let red = CGFloat(arc4random_uniform(255))/CGFloat(255.0)
let green = CGFloat(arc4random_uniform(255))/CGFloat(255.0)
let blue = CGFloat(arc4random_uniform(255))/CGFloat(255.0)
let alpha = CGFloat(arc4random_uniform(255))/CGFloat(255.0)

// 随机颜色
let randomColor = UIColor.init(red:red, green:green, blue:blue , alpha: alpha)



//地图标注物类型
enum AnnotationShowType {
    case NONE
    case STATION
    case PARKING
    case CAR
}
