pipeline {
    agent {
        dockerfile {}
    }

    stages {
        stage('Build') {
            steps {
                echo 'Build OK'
            }
        }
    }
    post {
         always {
             step([$class: 'TelegramBotBuilder', message: "${currentBuild.currentResult} -- ${env.JOB_NAME} -- ${env.CHANGE_URL}\nDuration: ${currentBuild.durationString}\nDetails: ${BUILD_URL}"])
         }
    }
}