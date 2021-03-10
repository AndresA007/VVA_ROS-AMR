#!/bin/bash

#------------------------------------------------------------------------------
# Copyright (c) 2020, Andres A. <andres.arboleda AT gmail DOT com>
# All rights reserved.
# 
# Released under the BSD License.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. All advertising materials mentioning features or use of this software
#    must display the following acknowledgement:
#    This product includes software developed by the <COPYRIGHT HOLDER>.
# 4. Neither the name of the <COPYRIGHT HOLDER> nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY <COPYRIGHT HOLDER> ''AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# Created by:
# andres.arboleda AT gmail DOT com, (july/2020)
#------------------------------------------------------------------------------

# Script to remove non-english special characters and accent marks from data sets
# and to construct the vocabulary.txt file

WORKDIR=/home/theuser/AAData/Documents2/ROS/Tutoriales/DeepSpeechWorkDir/

TRAIN_DATASET=${WORKDIR}/DataSets/M-AILABS_Corpus_es_ES/es_ES/es_ES_train.csv
DEV_DATASET=${WORKDIR}/DataSets/M-AILABS_Corpus_es_ES/es_ES/es_ES_dev.csv
TEST_DATASET=${WORKDIR}/DataSets/M-AILABS_Corpus_es_ES/es_ES/es_ES_test.csv

KENLM_BIN_PATH=${WORKDIR}/kenlm/build/bin/
GENERATE_TRIE_BINARY_PATH=${WORKDIR}/DeepSpeech-0.6.1_WorkDir/native_client.amd64.cpu.linux/

WAV_FILES_PREFIX=${WORKDIR}/DataSets/M-AILABS_Corpus_es_ES/es_ES/by_book/
WAV_FILES_START_KEYWORD=by_book/


# Temp file names
TRAIN_DATASET_TEMP1=`basename ${TRAIN_DATASET} | cut -d. -f1`_temp1.csv
DEV_DATASET_TEMP1=`basename ${DEV_DATASET} | cut -d. -f1`_temp1.csv
TEST_DATASET_TEMP1=`basename ${TEST_DATASET} | cut -d. -f1`_temp1.csv

TRAIN_DATASET_TEMP2=`echo ${TRAIN_DATASET_TEMP1} | sed 's/_temp1/_temp2/g'`
DEV_DATASET_TEMP2=`echo "${DEV_DATASET_TEMP1}" | sed 's/_temp1/_temp2/g'`
TEST_DATASET_TEMP2=`echo "${TEST_DATASET_TEMP1}" | sed 's/_temp1/_temp2/g'`

TRAIN_DATASET_TEMP3=`echo ${TRAIN_DATASET_TEMP1} | sed 's/_temp1/_temp3/g'`
DEV_DATASET_TEMP3=`echo "${DEV_DATASET_TEMP1}" | sed 's/_temp1/_temp3/g'`
TEST_DATASET_TEMP3=`echo "${TEST_DATASET_TEMP1}" | sed 's/_temp1/_temp3/g'`

TRAIN_DATASET_OUT=`echo ${TRAIN_DATASET_TEMP1} | sed 's/_temp1/_clean/g'`
DEV_DATASET_OUT=`echo "${DEV_DATASET_TEMP1}" | sed 's/_temp1/_clean/g'`
TEST_DATASET_OUT=`echo "${TEST_DATASET_TEMP1}" | sed 's/_temp1/_clean/g'`

TRAIN_DATASET_TEMP4=`echo ${TRAIN_DATASET_TEMP1} | sed 's/_temp1/_temp4/g'`
DEV_DATASET_TEMP4=`echo "${DEV_DATASET_TEMP1}" | sed 's/_temp1/_temp4/g'`
TEST_DATASET_TEMP4=`echo "${TEST_DATASET_TEMP1}" | sed 's/_temp1/_temp4/g'`

# Remove the .wav filename and the file size from the datasets .csv files:
cut -d, -f3 ${TRAIN_DATASET} > ${TRAIN_DATASET_TEMP1}
cut -d, -f3 ${DEV_DATASET} > ${DEV_DATASET_TEMP1}
cut -d, -f3 ${TEST_DATASET} > ${TEST_DATASET_TEMP1}

# Keep the .wav filename and the file size in a temp file:
cut -d, -f1,2 ${TRAIN_DATASET} | sed "s%${WAV_FILES_START_KEYWORD}%${WAV_FILES_PREFIX}%g" > ${TRAIN_DATASET_TEMP2}
cut -d, -f1,2 ${DEV_DATASET} | sed "s%${WAV_FILES_START_KEYWORD}%${WAV_FILES_PREFIX}%g" > ${DEV_DATASET_TEMP2}
cut -d, -f1,2 ${TEST_DATASET} | sed "s%${WAV_FILES_START_KEYWORD}%${WAV_FILES_PREFIX}%g" > ${TEST_DATASET_TEMP2}

# Remove punctuation, convert to lower-case and remove accent marks from the transcriptions:
sed $'s/[^[:alnum:][:blank:]]//g' ${TRAIN_DATASET_TEMP1} | \
sed 's/[A-Z]/\L&/g' | \
sed 's/á/a/g' | \
sed 's/é/e/g' | \
sed 's/í/i/g' | \
sed 's/ó/o/g' | \
sed 's/ú/u/g' | \
sed 's/ñ/n/g' | \
sed 's/¡//g' | \
sed 's/¿//g' \
> ${TRAIN_DATASET_TEMP3}

sed $'s/[^[:alnum:][:blank:]]//g' ${DEV_DATASET_TEMP1} | \
sed 's/[A-Z]/\L&/g' | \
sed 's/á/a/g' | \
sed 's/é/e/g' | \
sed 's/í/i/g' | \
sed 's/ó/o/g' | \
sed 's/ú/u/g' | \
sed 's/ñ/n/g' | \
sed 's/¡//g' | \
sed 's/¿//g' \
> ${DEV_DATASET_TEMP3}

sed $'s/[^[:alnum:][:blank:]]//g' ${TEST_DATASET_TEMP1} | \
sed 's/[A-Z]/\L&/g' | \
sed 's/á/a/g' | \
sed 's/é/e/g' | \
sed 's/í/i/g' | \
sed 's/ó/o/g' | \
sed 's/ú/u/g' | \
sed 's/ñ/n/g' | \
sed 's/¡//g' | \
sed 's/¿//g' \
> ${TEST_DATASET_TEMP3}

# Assemble again the .csv datasets including the clean transcription:
paste -d, ${TRAIN_DATASET_TEMP2} ${TRAIN_DATASET_TEMP3} > ${TRAIN_DATASET_OUT}
paste -d, ${DEV_DATASET_TEMP2} ${DEV_DATASET_TEMP3} > ${DEV_DATASET_OUT}
paste -d, ${TEST_DATASET_TEMP2} ${TEST_DATASET_TEMP3} > ${TEST_DATASET_OUT}

# Generate the vocabulary file:
sed 1d ${TRAIN_DATASET_TEMP3} > ${TRAIN_DATASET_TEMP4}
sed 1d ${DEV_DATASET_TEMP3} > ${DEV_DATASET_TEMP4}
sed 1d ${TEST_DATASET_TEMP3} > ${TEST_DATASET_TEMP4}
cat ${TRAIN_DATASET_TEMP4} ${DEV_DATASET_TEMP4} ${TEST_DATASET_TEMP4} > vocabulary.txt

# Generate the Language Model (LM)
${KENLM_BIN_PATH}/lmplz --text vocabulary.txt --arpa lm.arpa --order 4
${KENLM_BIN_PATH}/build_binary -a 255 -q 8 trie lm.arpa lm.binary
${GENERATE_TRIE_BINARY_PATH}/generate_trie ./alphabet.txt ./lm.binary ./trie

# Delete temp files:
rm ${TRAIN_DATASET_TEMP1} ${TRAIN_DATASET_TEMP2} ${TRAIN_DATASET_TEMP3} ${TRAIN_DATASET_TEMP4}
rm ${DEV_DATASET_TEMP1} ${DEV_DATASET_TEMP2} ${DEV_DATASET_TEMP3} ${DEV_DATASET_TEMP4}
rm ${TEST_DATASET_TEMP1} ${TEST_DATASET_TEMP2} ${TEST_DATASET_TEMP3} ${TEST_DATASET_TEMP4}


