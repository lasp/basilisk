import sys

if __name__ == "__main__":
     moduleOutputPath = sys.argv[1]
     headerinputPath = sys.argv[2]
     structType = sys.argv[3].split('Payload')[0]
     baseDir = sys.argv[4]

     swigTemplateFile = 'msgInterfacePy.i.in'

     swigFid = open(swigTemplateFile, 'r')
     swigTemplateData = swigFid.read()
     swigFid.close()

     moduleFileOut = open(moduleOutputPath, 'w')
     moduleFileOut.write(swigTemplateData.format(type=structType, baseDir=baseDir))
     moduleFileOut.close()
