import time
import isystem.connect as ic

connectionMgr = ic.ConnectionMgr()
connectionMgr.connectMRU('')

trdFileName = "OS_profiler.trd"

debugConfig = ic.CDebugFacade(connectionMgr)
profileConfig = ic.CProfilerController2(connectionMgr, trdFileName, "w")
ideConfig = ic.CIDEController(connectionMgr)

debugConfig.run()

# Wait that all threads are really started.
time.sleep(3)

eval = debugConfig.evaluate(ic.IConnectDebug.fRealTime, "_kernel.threads")
threadNames = []
threadsAddresses = []

nextThread = "->next_thread"
cnt = 0

while(1):
    eval = debugConfig.evaluate(ic.IConnectDebug.fRealTime, "_kernel.threads" + nextThread * cnt)
    if(eval.getAddress().m_aAddress == 0):
        break
    name = debugConfig.evaluate(ic.IConnectDebug.fRealTime, "_kernel.threads" + nextThread * cnt + "->name")
    nameStr = name.getResult(ic.CValueType.EChar).strip('"')
    threadAddress = eval.getAddress().m_aAddress
    cnt += 1
    threadNames.append(nameStr)
    threadsAddresses.append(threadAddress)


profileConfig.setAnalysisType(0, ic.CProfilerController2.EProfiler)
profileConfig.setProfilingSections(0, isProfileCode=False, isProfileData=True, isProfileOSObjects=False, isProfileAUX=False)

'''
Create header file for thread addresses
'''

with open("thread_names.h", "w") as file:
    for index, name in enumerate(threadNames):
        name = name.upper()
        name = name.replace(" ", "_")
        strToWrite = "#define {} {}\n".format(name.upper(), hex(threadsAddresses[index]))
        file.writelines(strToWrite)

profileConfig.addVariable(0, "_kernel.cpus[0].current", ic.CProfilerController2.EDATState)

description = "/Document/{}/Trigger.Items[0].Processes[0].Profiler.DataArea[0].Description".format(trdFileName)
ideConfig.setOption(description, "CPU_0")

define = "/Document/{}/Trigger.Items[0].Processes[0].Profiler.DataArea[0].Definition".format(trdFileName)
ideConfig.setOption(define, "define")

headerFile = "/Document/{}/Trigger.Items[0].Processes[0].Profiler.DataArea[0].DefinitionLocation".format(trdFileName)
ideConfig.setOption(headerFile, "thread_names.h")