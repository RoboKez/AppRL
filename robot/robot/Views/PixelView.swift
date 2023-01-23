//
//  PixelView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 20/07/2022.
//

import SwiftUI

struct PixelView: View {
//    @EnvironmentObject var rl: RL
    let data: [[Float]]
    let xMinID:Float
    let xMaxID:Float
    let yMinID:Float
    let yMaxID:Float
    
    var title: String = "My Image"
    var Xlabel: String = "x axis"
    var Ylabel: String = "y axis"
    var imageSize: CGFloat = 200
    var minHeat: Float = 0
    var maxHeat: Float = 0

    
    
    private var XlabelRange:String{
        return Xlabel + "  [" + String(xMinID) + " -> " + String(xMaxID) + "]"
    }
    
    private var YlabelRange:String{
        return Ylabel + "  [" + String(yMinID) + " -> " + String(yMaxID) + "]"
    }
    
    
    private var findMinMax:[Float]{

        var maxV = data[0][0]
        var minV = data[0][0]
        var minGX:Float = 0
        var maxGX:Float = 0
        var minGY:Float = 0
        var maxGY:Float = 0
    
        
        
        for i in 0...data.count-1 {
            for ii in 0...data[0].count-1 {
                if (data[i][ii] > maxV){
                    maxV = data[i][ii]
                    maxGX = Float(i)
                    maxGY = Float(ii)
                } else if (data[i][ii] < minV){
                    minV = data[i][ii]
                    minGX = Float(i)
                    minGY = Float(ii)
                }
            }
            
        }
        
        let varOx:Float = xMinID + (maxGX/Float(data.count-1)) * (xMaxID - xMinID)
        let varOy:Float = yMinID + (maxGY/Float(data[0].count-1)) * (yMaxID - yMinID)
        
        if(minHeat != maxHeat){
            minV = minHeat
            maxV = maxHeat
        }
        
        
        let a = [minV, maxV, minGX, maxGX, minGY, maxGY, varOx, varOy]
        print(varOx)
//        if (bruteForce){
//            print(rl.bestPI[0])
//            $rl.bestPI = [varOx, varOy]
//        }
        
        return a
    }

    var body: some View {
        let arr = findMinMax
        HStack{
            Text(YlabelRange).rotationEffect(.degrees(-90))
                .fixedSize()
                .frame(width: 10, height: 180)
                .font(.caption2)
            
            VStack{
                let t = title + "\nOb 0:" + String(arr[6]) + " Ob 1:" + String(arr[7])
                Text(t).font(.headline)
                    .fontWeight(.light)
                key.frame(width: imageSize, height: 20, alignment: .center)
                    .font(.subheadline)
                RGBgrid.frame(width: imageSize, height: imageSize, alignment: .center)
                Text(XlabelRange)
                    .font(.caption2)
            }
        }
        .padding()
//        .background(Color.purple)
        .background(.ultraThinMaterial)
//        .border(.green)
        
    }
}


extension PixelView {
    private var RGBgrid: some View {
        
        GeometryReader { g in
            let arr = findMinMax
            let minVal = arr[0]
            let maxVal = arr[1]
            let minGX = arr[2]
            let maxGX = arr[3]
            let minGY = arr[4]
            let maxGY = arr[5]
            
            let xSize:Int = data[0].count
            let ySize:Int = data.count
            let pixels:Int = xSize * ySize
            let x_dif = (g.size.width) / Double(xSize)
            let y_dif = (g.size.height) / Double(ySize)
            
            
            ForEach((0...pixels-1), id: \.self) {
                let i = $0
                let y = i % ySize
                let x = (i - y)/xSize
                
                let x_start:Double = x_dif*Double(x)
                let x_end:Double = x_start + x_dif

                let y_start:Double = y_dif*Double(y)
                let y_end:Double = y_start + y_dif

//                let ratio = 2 * (data[x][y] - xMinID) / ((xMaxID - xMinID)+0.000001)
                let ratio = 2 * (data[x][y] - minVal) / ((maxVal - minVal)+0.000001)
                let b_ = (max(0, 255*(1 - ratio)))
                let r_ = (max(0, 255*(ratio - 1)))
                let g_ = (255) - b_ - r_

                Path { p in
                    p.move(to: CGPoint(x: x_start, y: y_start))  // TL
                    p.addLine(to: CGPoint(x: x_end, y: y_start)) // BR
                    p.addLine(to: CGPoint(x: x_end, y: y_end))   // TR
                    p.addLine(to: CGPoint(x: x_start, y: y_end)) // BL
                    p.closeSubpath()
                }
                .stroke(.black, lineWidth: 2)

                Path { p in
                    p.move(to: CGPoint(x: x_start, y: y_start))  // TL
                    p.addLine(to: CGPoint(x: x_end, y: y_start)) // BR
                    p.addLine(to: CGPoint(x: x_end, y: y_end))   // TR
                    p.addLine(to: CGPoint(x: x_start, y: y_end)) // BL
                    p.closeSubpath()
                }
                .fill(Color(red: Double(r_)/255, green: Double(g_)/255, blue: Double(b_)/255))
                
            }
            
            Path { p in
                let x_start:Double = Double(maxGX) * (g.size.width) / Double(xSize)
                let x_end:Double = x_start + x_dif
                
                let y_start:Double = Double(maxGY) * (g.size.height) / Double(ySize)
                let y_end:Double = y_start + y_dif
                
                p.move(to: CGPoint(x: x_start, y: y_start))  // TL
                p.addLine(to: CGPoint(x: x_end, y: y_start)) // BR
                p.addLine(to: CGPoint(x: x_end, y: y_end))   // TR
                p.addLine(to: CGPoint(x: x_start, y: y_end)) // BL
                p.closeSubpath()
            }
            .stroke(.yellow, lineWidth: 2)
            
            Path { p in
                let x_start:Double = Double(minGX) * (g.size.width) / Double(xSize)
                let x_end:Double = x_start + x_dif
                
                let y_start:Double = Double(minGY) * (g.size.height) / Double(ySize)
                let y_end:Double = y_start + y_dif
                
                p.move(to: CGPoint(x: x_start, y: y_start))  // TL
                p.addLine(to: CGPoint(x: x_end, y: y_start)) // BR
                p.addLine(to: CGPoint(x: x_end, y: y_end))   // TR
                p.addLine(to: CGPoint(x: x_start, y: y_end)) // BL
                p.closeSubpath()
            }
            .stroke(.yellow, lineWidth: 2)
            
        }.rotationEffect(.degrees(180))
         .rotation3DEffect(.degrees(180), axis: (x:0, y:1, z:0))
        
    }
}

extension PixelView {
    private var key: some View {
        
        GeometryReader { g in
            let arr = findMinMax
            let minVal = arr[0]
            let maxVal = arr[1]
            
            // Boarder
            Path { p in

                let TL = CGPoint(x:0, y:0)
                let TR = CGPoint(x:g.size.width, y:0)
                let BR = CGPoint(x:g.size.width, y:g.size.height)
                let BL = CGPoint(x:0, y:g.size.height)
                    
                p.move(to: TL)
                p.addLine(to: TR)
                p.addLine(to: BR)
                p.addLine(to: BL)
                p.closeSubpath()
            }
            .stroke(.black, lineWidth: 2)
            let n = data[0].count
            let nSquares:Double = 100
            let x_dif = (g.size.width) / (Double(data[0].count) * nSquares)
            let loopCount = n*Int(nSquares)-1
            ForEach((0...loopCount), id: \.self) {

                let x_start:Double = x_dif*Double($0)
                let x_end:Double = x_start + x_dif

                let y_start:Double = 0
                let y_end:Double = g.size.height
                
                let ratio = Float(2)*Float($0)/Float(loopCount)
                let b_ = (max(0, 255*(1 - ratio)))
                let r_ = (max(0, 255*(ratio - 1)))
                let g_ = (255) - b_ - r_


                Path { p in
                    p.move(to: CGPoint(x: x_start, y: y_start))  // TL
                    p.addLine(to: CGPoint(x: x_end, y: y_start)) // BR
                    p.addLine(to: CGPoint(x: x_end, y: y_end))   // TR
                    p.addLine(to: CGPoint(x: x_start, y: y_end)) // BL
                    p.closeSubpath()
                }
                .fill(Color(red: Double(r_)/255, green: Double(g_)/255, blue: Double(b_)/255))
            }
            HStack{
                Text(String(minVal))
                Spacer()
                Text(String(maxVal))
            }
        }
    }
}
    
