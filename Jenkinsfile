pipeline {
  agent {
    dockerfile {
      reuseNode true
    }
  }

  stages {
    stage('Build') {
      steps {
        echo 'Build OK'
      }
    }
    stage('Test') {
      steps {
        echo 'Test OK'
      }
    }
    stage('Submit') {
      when {
        anyOf {
          branch 'lightning'
          branch 'final'
        }
      }
      environment {
        SUBMISSION_SSH_URL = 'icfpc@icfpc.gnolltech.org:public/2018/'
        SUBMISSION_PASSWORD = '83a5a58b38d74178b43b65caeef23500'
        BUILD_NAME = "${GIT_BRANCH}-${BUILD_NUMBER}-${GIT_COMMIT}"
      }
      steps {
        sshagent (credentials: ['DEPLOYMENT_KEY']) {
          sh "./make-submission.sh ${BUILD_NAME} ${SUBMISSION_PASSWORD} "
          sh "scp -q -oStrictHostKeyChecking=no ${BUILD_NAME}.zip ${BUILD_NAME}.zip.hash ${SUBMISSION_SSH_URL}"
        }
      }
    }
  }
  post {
    always {
      step([$class: 'TelegramBotBuilder', message: "${currentBuild.currentResult} -- ${env.JOB_NAME} -- ${env.CHANGE_URL}\nDuration: ${currentBuild.durationString}\nDetails: ${BUILD_URL}"])
    }
  }
}
