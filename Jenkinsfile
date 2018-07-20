pipeline {
  environment {
    SUBMISSION_SSH_URL = 'icfpc@icfpc.gnolltech.org:public/2018/'
    SUBMISSION_PASSWORD = 'N4yqRa2qrqNy'
  }
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
      steps {
        sshagent (credentials: ['DEPLOYMENT_KEY']) {
          sh "touch build-${GIT_BRANCH}-test"
          sh "scp -oStrictHostKeyChecking=no build-${GIT_BRANCH}-test ${SUBMISSION_SSH_URL}"
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
