//
//  NNSearchTextField.swift
//  NNMintFurniture
//
//  Created by 任晓亮 on 2018/09/27.
//  Copyright © 2018年 xiaoliang.Ren. All rights reserved.
//

import UIKit

class NNSearchTextField: UITextField {
    
    // MARK: - 初始化 NNSearchTextField
    override init(frame: CGRect) {
        super .init(frame: frame)
        leftView = leftV
        leftViewMode = UITextFieldViewMode.always
        placeholder = "请输入订单号:"
        background = UIImage.init(named: "sousuo")
        font = UIFont.systemFont(ofSize: 14)
        returnKeyType = UIReturnKeyType.search
        enablesReturnKeyAutomatically = true
    }
    
    // MARK: - 懒加载左边的搜索图片
    private lazy var leftV: UIView = {
        let leftV = UIView()
        let leftImage = UIImageView()
        let image = UIImage.init(named: "fangdajing")
        leftV.frame = CGRect(x: 5, y: 0, width: (image?.size.width)! + 15, height: (image?.size.height)!)
        leftImage.frame = CGRect(x: 5, y: 0, width: (image?.size.width)!, height: (image?.size.height)!)
        leftImage.image = image
        leftV.addSubview(leftImage)
        return leftV
    }()
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
