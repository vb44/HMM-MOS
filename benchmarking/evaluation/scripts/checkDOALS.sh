# *****************************************************************************
# Evaluates a DOALS sequence.
# | -------------------------------|
# | Seq | Sequence name            |
# |--------------------------------|
# |  1  | hauptgebaeude/sequence_1 |
# |  2  | hauptgebaeude/sequence_2 |
# |  3  | niederdorf/sequence_1    |
# |  4  | niederdorf/sequence_2    |
# |  5  | shopville/sequence_1     |
# |  6  | shopville/sequence_2     |
# |  7  | station/sequence_1       |
# |  8  | station/sequence_2       |
# | -------------------------------|
# *****************************************************************************
seqNum=1
minRange=0.5
maxRange=20

gtFilePath=/pathToSequenceFolder/hauptgebaeude/sequence_1/indices.csv
estFilePath=/pathToResultsFolder/test_seq1
scanFolderPath=/pathToSequenceFolder/hauptgebaeude/sequence_1/binFiles/

../build/evalIndFile $gtFilePath $estFilePath $scanFolderPath $seqNum $minRange $maxRange