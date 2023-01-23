//
//  ExhaustiveView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 20/07/2022.
//

import SwiftUI

struct ExhaustiveView: View {
    @EnvironmentObject var rl: RL
    @StateObject var control = Control()
    @State private var minP:Float = 0
    @State private var maxP:Float = 10
    @State private var minI:Float = 0
    @State private var maxI:Float = 10
    @State private var P:Float = 0
    @State private var I:Float = 0
    @State private var n:Int = 10
    @State private var searched:Bool = false
    @State private var searched2:Bool = false
    
    @State var data:[[Float]] = [[]]
//    @State var dataO:[[Float]] = [[]]
    
    var body: some View {
        VStack{
            HStack{
                    Text("Exhaustive Search (...within given precision)")
                        .fontWeight(.ultraLight)
                        .foregroundColor(Color.green)
                        .multilineTextAlignment(.center)
                        .padding(.all,10)
                        .font(.title)
                        .shadow(color: .green, radius: 10)
                        .padding()

                    
                Spacer()
                Button(action: {
                    rl.goHome()
                }, label: {Label("Home", systemImage: "house")
                    .foregroundColor(Color.green)})
                    .padding()
            }
            
            Divider()
            
            VStack{
                HStack{
                    TextField("Enter your Minimum Proportial Gain", value: $minP, format: .number)
                                    .textFieldStyle(.roundedBorder)

                    Text("Min P:\t \(minP)")
                }.padding()
                
                HStack{
                    TextField("Enter your Maximum Proportial Gain", value: $maxP, format: .number)
                                    .textFieldStyle(.roundedBorder)

                    Text("Max P:\t \(maxP)")
                }.padding()
                
                
                HStack{
                    TextField("Enter your Minimum Intergral Gain", value: $minI, format: .number)
                                    .textFieldStyle(.roundedBorder)

                    Text("Min I:\t \(minI)")
                }.padding()
                
                HStack{
                    TextField("Enter your Maximum Intergral Gain", value: $maxI, format: .number)
                                    .textFieldStyle(.roundedBorder)
    
                    Text("Max I:\t \(maxI)")
                }.padding()
                
                
                HStack{
                    TextField("Enter your Grid dimentions", value: $n, format: .number)
                                    .textFieldStyle(.roundedBorder)

                    Text("Grid search dims \(n) x \(n)")
                }.padding()
            }
            
            Divider()
            
            
            Button(action: {
                setFramerate(1)
                ExhaustiveSearch(minP, maxP, minI, maxI, Int32(n), Int32(2))  //0PD 1P 2PI 3PID
                let tmp = getExhastiveData()!;
                let n = Int(getExhaustiveSize())
                data = Array(repeating: Array(repeating: 0, count: n), count: n)
                for i in 0...n-1 {
                    for j in 0...n-1 {
                        data[i][j] = tmp[j + i * n]
                    }
                }
                searched = true;
                setFramerate(120)
                
            }, label: {Label("Brute Force", systemImage: "tornado")
                .foregroundColor(Color.green)})
                .padding()
            
            Divider()
            if searched {
                ScrollView{
                    PixelView(data: data,
                              xMinID: minP,
                              xMaxID: maxP,
                              yMinID: minI,
                              yMaxID: maxI,
                              title:"Brute Force Search",
                              Xlabel: "kP",
                              Ylabel: "kI")
                    
                    
                    HStack{
                        TextField("P", value: $P, format: .number)
                                        .textFieldStyle(.roundedBorder)
                        Text("P:\t \(P)")
                    }
                        HStack{
                        TextField("I", value: $I, format: .number)
                                        .textFieldStyle(.roundedBorder)
                        Text("I:\t \(P)")
                    }
                        
                    if searched2 {
                        PixelView(data: rl.PI,
                                  xMinID: rl.probeMinPI,
                                  xMaxID: rl.probeMaxPI,
                                  yMinID: rl.probeMinPI,
                                  yMaxID: rl.probeMaxPI,
                                  title:"PI Policy",
                                  Xlabel: "kP",
                                  Ylabel: "kI",
                                  imageSize: 250,
                                  minHeat: -1,
                                  maxHeat: 1)
                        
        
                    }
                }.padding()
                
                
                
                Button(action: {
                    rl.ProbePI(kP:P, kI:I)
                    searched2 = true;
                    
                    
//                    setFramerate(1)
                    // grid search each P and I same as NN probe
                    
                }, label: {Label("Show Policy for bute force found P I", systemImage: "map")
                    .foregroundColor(Color.green)})
                    .padding()
                
            } else {
                Text("Image will show when exhaused").padding()
            }
            Spacer()

        }

    }
}


