//
//  CustomOverlay.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/5.
//  Copyright © 2015年 baidu. All rights reserved.
//

import Foundation

class CustomOverlay: BMKPolyline {
    
    init(points: [BMKMapPoint]?, count: Int) {
        super.init()
        if points != nil {
            setPolylineWithPoints(UnsafeMutablePointer<BMKMapPoint>(mutating: points!), count: count)
        }
    }
    
}
