//
//  CustomOverlayView.swift
//  BMKSwiftDemo
//
//  Created by wzy on 15/11/5.
//  Copyright © 2015年 baidu. All rights reserved.
//

import UIKit

class CustomOverlayView: BMKOverlayGLBasicView {
    var customOverlay: CustomOverlay!
    
    override func glRender() {
        if customOverlay == nil {
            return;
        }

        let fillColor = UIColor(red: 1, green: 0, blue: 0, alpha: 0.5)
        renderRegion(withPoints: customOverlay.points, pointCount: UInt(customOverlay.pointCount), fill: fillColor, usingTriangleFan: true)
    }
    
}
