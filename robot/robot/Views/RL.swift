//
//  RL.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 10/05/2022.
//
import SwiftUI

class RL: ObservableObject {
    @Published var autorun = false
    @Published var portConnected = false
    @Published var curPort = "Select USB port..."
    @Published var newExp = true
    @Published var curTitle = "RL Investigation"
    @Published var userQuit:Int32 = 0
    
    @Published var view = 0
    @Published var envs: [String] = []
    @Published var files: [String] = []
    @Published var ports: [String] = []
    @Published var path: String = ""

    @Published var running: Bool = false
    @Published var real: Bool = false
    
    @Published var x1:[Int] = []
    @Published var x2:[Int] = []
    @Published var x3:[Int] = []
    @Published var x4:[Int] = []
    @Published var x5:[Int] = []

    @Published var y1s:[Float] = []
    @Published var y2s:[Float] = []
    @Published var y3s:[Float] = []
    @Published var y4s:[Float] = []
    @Published var y5s:[Float] = []
    
    @Published var y1d:[Float] = []
    @Published var y2d:[Float] = []
    @Published var y3d:[Float] = []
    @Published var y4d:[Float] = []
    @Published var y5d:[Float] = []
    
    @Published var x1_pid:[Int] = []
    @Published var y1_pid:[Float] = []
    @Published var x2_pid:[Int] = []
    @Published var y2_pid:[Float] = []
    
    @Published var comColour: [Color] = [.green, .blue, .red, .yellow, .cyan]
    @Published var comNames:[String] = ["tmp", "None", "None", "None", "None"]
    @Published var comForm:[Bool] = [true, false, false, false, false]
    
    @Published var comLr:[Float] = [0.0003, -1, -1, -1, -1]
    @Published var comSeed:[Int32] = [1, -1, -1, -1, -1]
    @Published var comH1:[Int32] = [8, -1, -1, -1, -1]
    @Published var comH2:[Int32] = [8, -1, -1, -1, -1]
    @Published var comEpochs:[Int32] = [10, -1, -1, -1, -1]
    @Published var comMinibatch:[Int32] = [64, -1, -1, -1, -1]
    @Published var comIterSample:[Int32] = [2048, -1, -1, -1, -1]
    @Published var comStepLimit:[Int32] = [200, -1, -1, -1, -1]
    @Published var comAdam:[Bool] = [true, false, false, false, false]
    @Published var comEnv:[String] = ["cartpoleClassic", "", "", "", ""]
    
    @Published var m_KP:Float = 4.57
    @Published var m_KI:Float = 5.1
    @Published var m_KD:Float = 0.09
    @Published var isEditing:Bool = false
    @Published var m_targetPitch:Float = 0.016
    @Published var inter_limit:Float = 10.0
    @Published var err:Float = 0.0
    @Published var sub_file:String = "c"
    @Published var iter:Int32 = 0
    @Published var actionFilter:Float = 0
    
    @Published var probeValues:[Float] = Array(repeating: 0, count: 441)
    @Published var probeValues2:[Float] = Array(repeating: 0, count: 441)
    @Published var probePolicy:[Float] = Array(repeating: 0, count: 441)
    @Published var probePolicy2:[Float] = Array(repeating: 0, count: 441)
    @Published var PixelProbeValues:[Float] = Array(repeating: 0, count: 441)
    @Published var PixelProbePolicy:[Float] = Array(repeating: 0, count: 441)
    @Published var PixelProbeValues2:[Float] = Array(repeating: 0, count: 441)
    @Published var PixelProbePolicy2:[Float] = Array(repeating: 0, count: 441)
    
    @Published var Values:[[Float]] = Array(repeating: Array(repeating: 0, count: 21), count: 21)
    @Published var Policy:[[Float]] = Array(repeating: Array(repeating: 0, count: 21), count: 21)
    @Published var Values2:[[Float]] = Array(repeating: Array(repeating: 0, count: 21), count: 21)
    @Published var Policy2:[[Float]] = Array(repeating: Array(repeating: 0, count: 21), count: 21)
    @Published var PI:[[Float]] = Array(repeating: Array(repeating: 0, count: 21), count: 21)
    
    
    @Published var probeMinV:Float = 0
    @Published var probeMaxV:Float = 0
    @Published var probeMinP:Float = 0
    @Published var probeMaxP:Float = 0
    @Published var probeMinPI:Float = 0
    @Published var probeMaxPI:Float = 0
    @Published var probeN:Int = 441
    @Published var showProbe:Bool = false
    @Published var annealFilter:Bool = false
    @Published var bestPI:[Float] = [0,0]
    @Published var sensorNoise:Float = 0
    @Published var startState:Int32 = 0
    
    
    let appResources = Bundle.main.resourcePath!
    
    init() {
        path = String(describing: appResources)
        envs = csvToArray(c: String(cString: listEnvs()!))
        files = csvToArray(c: String(cString: listJsonFiles(path)!))
    }
    
    func goHome(){
        clearData(GraphID: 0)
        clearData(GraphID: 1)
        clearData(GraphID: 2)
        clearData(GraphID: 3)
        clearData(GraphID: 4)
        files = csvToArray(c: String(cString: listJsonFiles(path)!))
        view = 0
    }
    
    func updateParameters(){
        setEnv(comEnv[0])
        setSeed(comSeed[0])
        setH1(comH1[0])
        setH2(comH2[0])
        setLr(comLr[0])
        setEpochs(comEpochs[0])
        setMinibatch(comMinibatch[0])
        setIterSamples(comIterSample[0])
        setStepLimit(comStepLimit[0])
        
        if comAdam[0]{
            setAdam(1);
        } else {
            setAdam(0);
        }

    }
    
    func clearData(GraphID: Int){
        if GraphID==0{
            comForm[0] = false
            x1 = []
            y1s = []
            y1d = []
            
        } else if GraphID==1{
            comNames[1] = "None"
            comForm[1] = false
            x2 = []
            y2s = []
            y2d = []
            
        } else if GraphID==2{
            comNames[2] = "None"
            comForm[2] = false
            x3 = []
            y3s = []
            y3d = []
            
        } else if GraphID==3{
            comNames[3] = "None"
            comForm[3] = false
            x4 = []
            y4s = []
            y4d = []
            
        } else if GraphID==4 {
            comNames[4] = "None"
            comForm[4] = false
            x5 = []
            y5s = []
            y5d = []
        }
    }
    
    func cancelExperiment(){
        swiftCancel()
        running = false
    }
    
    func closeExperiment(){
        swiftCancel()
        deleteAgentandEnv()
        running = false
        print("swift Cancellled")
    }
    
    func getEnvObs() -> [Float]{
        let tmp_ob = getEnvOb()!
        var ob: [Float] = []
        for j in 0...getObDim()-1 {
            ob.append(tmp_ob[Int(j)])
        }
        return ob
    }
    
    func getAgentObs() -> [Float]{
        let tmp_ob = getAgentOb()!
        var ob: [Float] = []
        for j in 0...getObDim()-1 {
            ob.append(tmp_ob[Int(j)])
        }
        return ob
    }
    
    func newExperiment(subAgent:String="c1"){
        running = true
//        agentReset(path, curPort, subAgent+".json", -1)
        rlReset()
    }
    func loadEnv(i: Int, subAgent:String="c1")  ->  Int{
        running = true
        if real{
         print("cur port", curPort)
        }
        setPathAndFile(path, comNames[0]+".json", comNames[0]+"-sub.json", Int32(-1))
        let i_start = loadNetwork(Int32(i), 0)
        getReturns(graphID: 0, fileName: comNames[0]+".json")
        return Int(i_start)
        
        
    }
    
    func MoCaploadAgent(i: Int)  ->  Int{
        running = true
        setPathAndFile(path, comNames[0]+".json", comNames[0]+"-sub.json", Int32(-1))
        
        let i_start = loadNetwork(Int32(i), 1)
        
        return Int(i_start)
    }
    
    func deleteExperiment(fileName: String){
        deleteJson(path, fileName)
        files = csvToArray(c: String(cString: listJsonFiles(path)!))
    }
    

    func ProbeNet(fileName: String, new:Bool=false, load:Bool=false){
            var tmpV_ = UnsafeMutablePointer<Float>.allocate(capacity: probeN)
            var tmpP_ = UnsafeMutablePointer<Float>.allocate(capacity: probeN)

//            if(load){
//                tmpV_ = getValueProbe(fileName, -1)!
//                tmpP_ = getPolicyProbe(fileName, -1)!
//            } else {
                tmpV_ = getNewValueProbe(0,0,1)!
                tmpP_ = getNewPolicyProbe(0,0,1)!
//            }
            
            let n = Int(sqrt(Double(probeN)))
            for i in 0...n-1 {
                for j in 0...n-1 {
                    Policy[i][j] = tmpP_[j + i * n]
                    Values[i][j] = tmpV_[j + i * n]
                }
            }
        
        var tmpV2_ = UnsafeMutablePointer<Float>.allocate(capacity: probeN)
        var tmpP2_ = UnsafeMutablePointer<Float>.allocate(capacity: probeN)
        
        if(!load && (comEnv[0]=="cartpoleClassic" || comEnv[0]=="Classic_Primary") ){
            
            tmpV2_ = getNewValueProbe(0,2,3)!
            tmpP2_ = getNewPolicyProbe(0,2,3)!
        }
        
        for i in 0...n-1 {
            for j in 0...n-1 {
                Policy2[i][j] = tmpP2_[j + i * n]
                Values2[i][j] = tmpV2_[j + i * n]
            }
        }
            
//            showProbe = true
            
            probeMaxV = 1.0 //probeValues.max()!
            probeMinV = -1.0 //probeValues.min()!
            
            probeMaxP = 1.0// probePolicy.max()!
            probeMinP = -1.0//probePolicy.min()!
    }
    
    
    func ProbePI(kP:Float, kI:Float){
        
        
        let tmpP_ = PIProbe(kP, kI)!
        probeMaxPI = tmpP_[0] //probeValues.max()!
        probeMinPI = tmpP_[0] //probeValues.min()!
        
        let n = Int(sqrt(Double(probeN)))
            for i in 0...n-1 {
                for j in 0...n-1 {
                    PI[i][j] = tmpP_[j + i * n]
                    if (PI[i][j] > probeMaxPI) {probeMaxPI = PI[i][j]}
                    else if (PI[i][j] < probeMinPI) {probeMinPI = PI[i][j]}
                }
            }
    }

    
    func getReturns(graphID: Int, fileName: String){
        print("get returns", fileName)
        
        let n = iterCount(path, fileName)
        let Steps = getSteps(fileName)!
        let Socs = getStocastic(fileName)!
        let Dets = getDeterministic(fileName)!
        
        let ch = Character(".")
        let result = fileName.split(separator: ch).map { String($0) }
        let file_title = result[0]

        var avgReturnCount: [Int] = [];
        var avgReturnSoc: [Float] = [];
        var avgReturnDet: [Float] = [];
        for j in 0...n-1 {
            avgReturnCount.append(Int(Steps[Int(j)]))
            avgReturnSoc.append(Socs[Int(j)])
            avgReturnDet.append(Dets[Int(j)])
        }

        if (graphID == 0){
            x1 = avgReturnCount
            y1s = avgReturnSoc
            y1d = avgReturnDet
            comNames[0] = file_title
            
        } else if (graphID == 1){
            x2 = avgReturnCount
            y2s = avgReturnSoc
            y2d = avgReturnDet
            comNames[1] = file_title
            
        } else if (graphID == 2){
            x3 = avgReturnCount
            y3s = avgReturnSoc
            y3d = avgReturnDet
            comNames[2] = file_title
            
        } else if (graphID == 3){
            x4 = avgReturnCount
            y4s = avgReturnSoc
            y4d = avgReturnDet
            comNames[3] = file_title
            
        } else if (graphID == 4){
            x5 = avgReturnCount
            y5s = avgReturnSoc
            y5d = avgReturnDet
            comNames[4] = file_title
        }
    }
    
    func getParams(graphID: Int, fileName: String, probe: Bool=true, first: Bool=true){
        print("getParams file", fileName)
        getReturns(graphID: graphID, fileName: fileName)
        comEnv[graphID] = String(cString: getEnv(path, fileName))
        comLr[graphID] = getLr(path, fileName)
        
        let tmp = getIntData(path, fileName)!;
        comH1[graphID] = tmp[0]
        comH2[graphID] = tmp[1]
        comEpochs[graphID] = tmp[2]
        comMinibatch[graphID] = tmp[3]
        comIterSample[graphID] = tmp[4]
        comStepLimit[graphID] = tmp[5]
        comAdam[graphID] = (tmp[6] != 0)
        comSeed[graphID] = tmp[7]
        comForm[graphID] = true
        
        if(!first){
            let maxiter = loadNetwork(Int32(-1), 1)
            loadNetwork(maxiter-1, 1)
        }
        
        if(probe){
            ProbeNet(fileName: fileName, load: first)
        }
        
    }
    
    func csvToArray(c: String) -> [String] {
        let ch = Character(",")
        let result = c.split(separator: ch).map { String($0) }
        return result;
    }
    
    func updatePorts(){
        portConnected = false;
        curPort = "Select USB port..."
        ports = csvToArray(c: String(cString: listPorts(path)!))
    }
    
    func realCheck(){
        if comEnv[0] == "trolley"{
            real = true
        } else {
            real = false
        }
    }
    
    func pidReset()->[Float]{
        let pid_ob_point = manualStart(path, curPort)!
        var pid_ob: [Float] = []
        for j in 0...0 {pid_ob.append(pid_ob_point[j])}
        return pid_ob
    }
    
    func pidStep(pidAct:Float, saveID:Int32)->[Float]{
        let pid_ob_point = manualStep(pidAct, saveID)!
        var pid_ob: [Float] = []
        for j in 0...0 {pid_ob.append(pid_ob_point[j])}
        return pid_ob
    }
    
    func SIReset()->[Float]{
        let si_ob_size = modelStart(path, comNames[0]+".json", -1, curPort)
        let si_obs = Array<Float>(repeating: 0, count: Int(si_ob_size))
        return si_obs
    }
    
}

class Control: ObservableObject {
    @Published var m_inverter:Bool = false
    
    @Published var m_KPa:Float = 4.69
    @Published var m_KIa:Float = 5.1
    @Published var m_KDa:Float = 0.09
    @Published var m_limIa:Float = 10.0
    
    @Published var m_KPv:Float = 0.03
    @Published var m_KIv:Float = 0
    @Published var m_KDv:Float = 0
    @Published var m_limIv:Float = 1.0
    
    @Published var m_tarPitch:Float = 0.0
    @Published var m_tarTorque:Float = 0.0
    @Published var m_prevEa:Float = 0.0
    @Published var m_prevEv:Float = 0.0
    
    @Published var m_prevIa:Float = 0.0
    @Published var m_prevIv:Float = 0.0
    
    @Published var m_Pa:Float = 0
    @Published var m_Ia:Float = 0
    @Published var m_Da:Float = 0
    
    @Published var m_Pv:Float = 0
    @Published var m_Iv:Float = 0
    @Published var m_Dv:Float = 0
    
    @Published var xtime:[Int] = []
    @Published var ya:[Float] = []
    @Published var yat:[Float] = []
    @Published var yv:[Float] = []
    @Published var yvt:[Float] = []
    
    @Published var m_Ea:Float = 0
    @Published var m_Ev:Float = 0
    @Published var m_running:Bool = false
    @Published var m_pitchOnly:Bool = true
    @Published var i:Int = 0
    
    
    func defaultParams(){
        m_KPa = 4.69
        m_KIa = 5.1
        m_KDa = 0.09
        m_limIa = 10.0
        m_KPv = 0.03
        m_KIv = 0
        m_KDv = 0
        m_limIv = 1.0
        m_tarPitch = 0.0
    }

    
    
    private func PitchPID(E:Float, Ts:Float)-> Float{
        m_Pa = m_KPa * E
        m_Da = m_KDa * (E - m_prevEa) / Ts
        m_Ia = m_KIa * (E + m_prevEa) * Ts * 0.5 + m_prevIa
        m_Ia = Limit(val:m_Ia, lowLim: -m_limIa, UpLim: m_limIa)
        
        m_tarTorque = m_Pa + m_Ia + m_Da
        m_prevEa = E
        m_prevIa = m_Ia
        
        return m_tarTorque
    }
    
    private func WheelPID(E:Float, Ts:Float)-> Float{
        m_Pv = m_KPv * E
        m_Dv = m_KDv * (E - m_prevEv) / Ts
        m_Iv = m_KIv * (E + m_prevEv) * Ts * 0.5 + m_prevIv
        m_Iv = Limit(val:m_Iv, lowLim: -m_limIv, UpLim: m_limIv)
        
        let tarPitch = m_Pv + m_Iv + m_Dv
        m_prevEv = E
        m_prevIv = m_Iv
        return tarPitch
    }
    
    func Cascade(tarVel:Float, curVel:Float, curPitch:Float, Ts:Float)-> Float{
        
        m_Ev = tarVel-curVel
        m_tarPitch = Limit(val: WheelPID(E: m_Ev, Ts: Ts), lowLim: -10, UpLim: 10)
        m_Ea = curPitch-m_tarPitch
        m_tarTorque = PitchPID(E: m_Ea, Ts: Ts)
        updateGraph(tarVel: tarVel, curVel: curVel, curPitch: curPitch)
        return m_tarTorque
    }
    
    
    func justStabalise(tarAng:Float, curPitch:Float, Ts:Float)-> Float{
        
        var inv:Float = 1.0
        if(m_inverter){
            inv = -1.0
        }
        
        m_tarPitch = tarAng
        
        m_Ea = curPitch*inv-m_tarPitch
        m_tarTorque = PitchPID(E: m_Ea, Ts: Ts)
        updateGraph(tarVel: 0, curVel: 0, curPitch: curPitch)
        return m_tarTorque
    }
    
    func Limit(val:Float, lowLim:Float, UpLim:Float)->Float{
        let valLimited = min(max(val, lowLim), UpLim)
        return valLimited
    }
    
    func Reset(){
        xtime = []
        ya = []
        yat = []
        yv = []
        yvt = []
        i = 0
        m_prevEa = 0.0
        m_prevEv = 0.0
        
        m_prevIa = 0.0
        m_prevIv = 0.0
        m_running = true
    }
    
    func updateGraph(tarVel:Float, curVel:Float, curPitch:Float){
        if xtime.count<100{
            // shift
            xtime.append(i)
            ya.append(curPitch)
            yat.append(m_tarPitch)
            
            yv.append(curVel)
            yvt.append(tarVel)
            
        } else {
            for j in 0...98{
                xtime[j] = xtime[j+1]
                ya[j] = ya[j+1]
                yat[j] = yat[j+1]
                yv[j] = yv[j+1]
                yvt[j] = yvt[j+1]
            }
            
            xtime[99] = i
            ya[99] = curPitch
            yat[99] = m_tarPitch
            yv[99] = curVel
            yvt[99] = tarVel
        }
        i+=1
    }
}


class Live: ObservableObject {
    
    @Published var xtime:[Int] = []
    @Published var ya:[Float] = []
    @Published var yat:[Float] = []
    @Published var m_running:Bool = false
    @Published var i:Int = 0
    @Published var yap:[Float] = []
    
    func Reset(){
        xtime = []
        ya = []
        yat = []
        yap = []
        i = 0
        m_running = true
    }
    
    func updateGraph(tarPitch:Float, curPitch:Float){
        if xtime.count<100{
            // shift
            xtime.append(i)
            ya.append(curPitch)
            yat.append(tarPitch)
            
        } else {
            for j in 0...98{
                xtime[j] = xtime[j+1]
                ya[j] = ya[j+1]
                yat[j] = yat[j+1]
            }
            xtime[99] = i
            ya[99] = curPitch
            yat[99] = tarPitch
        }
        i+=1
    }
    
    func updateGraph3(tarPitch:Float, curPitch:Float, predPitch:Float){
        if xtime.count<100{
            // shift
            xtime.append(i)
            ya.append(curPitch)
            yat.append(tarPitch)
            yap.append(predPitch)
            
        } else {
            for j in 0...98{
                xtime[j] = xtime[j+1]
                ya[j] = ya[j+1]
                yat[j] = yat[j+1]
                yap[j] = yap[j+1]
            }
            xtime[99] = i
            ya[99] = curPitch
            yat[99] = tarPitch
            yap[99] = predPitch
        }
        i+=1
    }
}
