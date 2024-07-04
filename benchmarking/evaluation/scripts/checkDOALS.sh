minRange=1
maxRange=20
range=range_$maxRange

seq=1
gtFilePath=/media/vb/Gift/DOALS/processed/hauptgebaeude/sequence_1/indices.csv
estFilePath=/home/vb/Documents/public_repositories/QCD_MOS/results/test_seq${seq}
scanFolderPath=/media/vb/VB/datasets/DOALS/hauptgebaeude/sequence_1/binFiles/
../build/evalIndFile $gtFilePath $estFilePath $scanFolderPath $seq $minRange $maxRange

seq=2
gtFilePath=/media/vb/Gift/DOALS/processed/hauptgebaeude/sequence_2/indices.csv
estFilePath=/home/vb/Documents/public_repositories/QCD_MOS/results/test_seq${seq}
scanFolderPath=/media/vb/VB/datasets/DOALS/hauptgebaeude/sequence_2/binFiles/
../build/evalIndFile $gtFilePath $estFilePath $scanFolderPath $seq $minRange $maxRange

seq=3
gtFilePath=/media/vb/Gift/DOALS/processed/niederdorf/sequence_1/indices.csv
estFilePath=/home/vb/Documents/public_repositories/QCD_MOS/results/test_seq${seq}
scanFolderPath=/media/vb/VB/datasets/DOALS/niederdorf/sequence_1/binFiles/
../build/evalIndFile $gtFilePath $estFilePath $scanFolderPath $seq $minRange $maxRange

seq=4
gtFilePath=/media/vb/Gift/DOALS/processed/niederdorf/sequence_2/indices.csv
estFilePath=/home/vb/Documents/public_repositories/QCD_MOS/results/test_seq${seq}
scanFolderPath=/media/vb/VB/datasets/DOALS/niederdorf/sequence_2/binFiles/
../build/evalIndFile $gtFilePath $estFilePath $scanFolderPath $seq $minRange $maxRange

seq=5
gtFilePath=/media/vb/Gift/DOALS/processed/shopville/sequence_1/indices.csv
estFilePath=/home/vb/Documents/public_repositories/QCD_MOS/results/test_seq${seq}
scanFolderPath=/media/vb/VB/datasets/DOALS/shopville/sequence_1/binFiles/
../build/evalIndFile $gtFilePath $estFilePath $scanFolderPath $seq $minRange $maxRange

seq=6
gtFilePath=/media/vb/Gift/DOALS/processed/shopville/sequence_2/indices.csv
estFilePath=/home/vb/Documents/public_repositories/QCD_MOS/results/test_seq${seq}
scanFolderPath=/media/vb/VB/datasets/DOALS/shopville/sequence_2/binFiles/
../build/evalIndFile $gtFilePath $estFilePath $scanFolderPath $seq $minRange $maxRange

seq=7
gtFilePath=/media/vb/Gift/DOALS/processed/station/sequence_1/indices.csv
estFilePath=/home/vb/Documents/public_repositories/QCD_MOS/results/test_seq${seq}
scanFolderPath=/media/vb/VB/datasets/DOALS/station/sequence_1/binFiles/
../build/evalIndFile $gtFilePath $estFilePath $scanFolderPath $seq $minRange $maxRange

seq=8
gtFilePath=/media/vb/Gift/DOALS/processed/station/sequence_2/indices.csv
estFilePath=/home/vb/Documents/public_repositories/QCD_MOS/results/test_seq${seq}
scanFolderPath=/media/vb/VB/datasets/DOALS/station/sequence_2/binFiles/
../build/evalIndFile $gtFilePath $estFilePath $scanFolderPath $seq $minRange $maxRange