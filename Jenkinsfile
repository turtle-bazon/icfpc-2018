pipeline {
  agent {
    dockerfile {
      reuseNode true
    }
  }

  stages {
    stage('Build') {
      steps {
        sh "make"
      }
    }
    stage('Test') {
      steps {
        sh "make test"
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
          sh "make BUILD_NAME=${BUILD_NAME} TEAM_ID=${SUBMISSION_PASSWORD} submission"
        }
      }
    }
  }
  post {
    always {
      script {
        def changeLog = "```";
        for (int i = 0; i < currentBuild.changeSets.size(); i++) {
          def entries = currentBuild.changeSets[i].items
          for (int j = 0; j < entries.length; j++) {
            def entry = entries[j]
            changeLog += " * \"${entry.msg}\" by ${entry.author}\n"
          }
        }
        changeLog += "```"

        step([$class: 'TelegramBotBuilder', message: "*${currentBuild.currentResult}*  ${env.JOB_NAME}\nChanges:\n${changeLog}[Build log](${BUILD_URL})"]);
      }
    }
  }
}
